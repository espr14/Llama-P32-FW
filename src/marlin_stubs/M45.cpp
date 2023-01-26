#include "PrusaGcodeSuite.hpp"
#include "M45.hpp"

// from G29
#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../feature/bedlevel/bedlevel.h"
#include "../../../module/motion.h"
#include "../../../module/planner.h"
#include "../../../module/stepper.h"
#include "../../../module/probe.h"
#include "../../queue.h"

#if HAS_DISPLAY
    #include "../../../lcd/ultralcd.h"
#endif

#if ENABLED(AUTO_BED_LEVELING_LINEAR)
    #include "../../../libs/least_squares_fit.h"
#endif

#if ABL_PLANAR
    #include "../../../libs/vector_3.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../../core/debug_out.h"

#if ENABLED(EXTENSIBLE_UI)
    #include "../../../lcd/extensible_ui/ui_api.h"
#endif

#if HOTENDS > 1
    #include "../../../module/tool_change.h"
#endif

#if ABL_GRID
    #if ENABLED(PROBE_Y_FIRST)
        #define PR_OUTER_VAR meshCount.x
        #define PR_OUTER_END abl_grid_points.x
        #define PR_INNER_VAR meshCount.y
        #define PR_INNER_END abl_grid_points.y
    #else
        #define PR_OUTER_VAR meshCount.y
        #define PR_OUTER_END abl_grid_points.y
        #define PR_INNER_VAR meshCount.x
        #define PR_INNER_END abl_grid_points.x
    #endif
#endif

#if ENABLED(G29_RETRY_AND_RECOVER)
    #define G29_RETURN(b) return b;
#else
    #define G29_RETURN(b) return;
#endif

// end from G29

void PrusaGcodeSuite::M45() {
    /// home XY if needed
    /// home Z at safe position (get reference height)
    /// cycle over 9 points
    /// go to point
    /// scan 32x32 array
    /// calculate center
    /// calculate XY skews
    /// pick median
    /// set XY skew

    /// G29 start
    constexpr bool seenQ = false;
    constexpr bool seenA = false;

    const bool no_action = seenA || seenQ,
               faux = no_action;

    // Don't allow auto-leveling without homing first
    // TODO do G28
    if (axis_unhomed_error())
        G29_RETURN(false);

        // Define local vars 'static' for manual probing, 'auto' otherwise
#define ABL_VAR

    ABL_VAR int verbose_level;
    ABL_VAR xy_pos_t probePos;
    ABL_VAR float measured_z;
    ABL_VAR bool dryrun, abl_should_enable;
    ABL_VAR xy_int_t probe_position_lf, probe_position_rb;
    ABL_VAR xy_float_t gridSpacing = { 0, 0 };

    constexpr xy_uint8_t abl_grid_points = { GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y };

    ABL_VAR float zoffset;

    /**
     * On the initial G29 fetch command parameters.
     */
    if (!g29_in_progress) {

        abl_should_enable = planner.leveling_active;

        const bool seen_w = parser.seen('W');
        if (seen_w) {
            if (!leveling_is_valid()) {
                SERIAL_ERROR_MSG("No bilinear grid");
                G29_RETURN(false);
            }

            const float rz = parser.seenval('Z') ? RAW_Z_POSITION(parser.value_linear_units()) : current_position.z;
            if (!WITHIN(rz, -10, 10)) {
                SERIAL_ERROR_MSG("Bad Z value");
                G29_RETURN(false);
            }

            const float rx = RAW_X_POSITION(parser.linearval('X', NAN)),
                        ry = RAW_Y_POSITION(parser.linearval('Y', NAN));
            int8_t i = parser.byteval('I', -1), j = parser.byteval('J', -1);

            if (!isnan(rx) && !isnan(ry)) {
                // Get nearest i / j from rx / ry
                i = (rx - bilinear_start.x + 0.5 * gridSpacing.x) / gridSpacing.x;
                j = (ry - bilinear_start.y + 0.5 * gridSpacing.y) / gridSpacing.y;
                LIMIT(i, 0, GRID_MAX_POINTS_X - 1);
                LIMIT(j, 0, GRID_MAX_POINTS_Y - 1);
            }
            if (WITHIN(i, 0, GRID_MAX_POINTS_X - 1) && WITHIN(j, 0, GRID_MAX_POINTS_Y)) {
                set_bed_leveling_enabled(false);
                z_values[i][j] = rz;
                set_bed_leveling_enabled(abl_should_enable);
                if (abl_should_enable)
                    report_current_position();
            }
            G29_RETURN(false);
        } // parser.seen('W')

        verbose_level = parser.intval('V');
        if (!WITHIN(verbose_level, 0, 4)) {
            SERIAL_ECHOLNPGM("?(V)erbose level implausible (0-4).");
            G29_RETURN(false);
        }

        dryrun = parser.boolval('D');

        zoffset = parser.linearval('Z');

        xy_probe_feedrate_mm_s = MMM_TO_MMS(parser.linearval('S', XY_PROBE_SPEED));

        const float x_min = probe_min_x(), x_max = probe_max_x(),
                    y_min = probe_min_y(), y_max = probe_max_y();

        if (parser.seen('H')) {
            const int16_t size = (int16_t)parser.value_linear_units();
            probe_position_lf.set(
                _MAX(X_CENTER - size / 2, x_min),
                _MAX(Y_CENTER - size / 2, y_min));
            probe_position_rb.set(
                _MIN(probe_position_lf.x + size, x_max),
                _MIN(probe_position_lf.y + size, y_max));
        } else {
            probe_position_lf.set(
                parser.seenval('L') ? (int)RAW_X_POSITION(parser.value_linear_units()) : _MAX(X_CENTER - (X_BED_SIZE) / 2, x_min),
                parser.seenval('F') ? (int)RAW_Y_POSITION(parser.value_linear_units()) : _MAX(Y_CENTER - (Y_BED_SIZE) / 2, y_min));
            probe_position_rb.set(
                parser.seenval('R') ? (int)RAW_X_POSITION(parser.value_linear_units()) : _MIN(probe_position_lf.x + X_BED_SIZE, x_max),
                parser.seenval('B') ? (int)RAW_Y_POSITION(parser.value_linear_units()) : _MIN(probe_position_lf.y + Y_BED_SIZE, y_max));
        }

        if (
            !position_is_reachable_by_probe(probe_position_lf)
            || !position_is_reachable_by_probe(probe_position_rb)) {
            SERIAL_ECHOLNPGM("? (L,R,F,B) out of bounds.");
            G29_RETURN(false);
        }

        // probe at the points of a lattice grid
        gridSpacing.set((probe_position_rb.x - probe_position_lf.x) / (abl_grid_points.x - 1),
            (probe_position_rb.y - probe_position_lf.y) / (abl_grid_points.y - 1));

        planner.synchronize();

        // Disable auto bed leveling during G29.
        // Be formal so G29 can be done successively without G28.
        if (!no_action)
            set_bed_leveling_enabled(false);

#if HAS_BED_PROBE
        // Deploy the probe. Probe will raise if needed.
        if (DEPLOY_PROBE()) {
            set_bed_leveling_enabled(abl_should_enable);
            G29_RETURN(false);
        }
#endif

        if (!faux)
            remember_feedrate_scaling_off();

        if (gridSpacing != bilinear_grid_spacing || probe_position_lf != bilinear_start) {
            // Reset grid to 0.0 or "not probed". (Also disables ABL)
            reset_bed_level();

            // Initialize a grid with the given dimensions
            bilinear_grid_spacing = gridSpacing.asInt();
            bilinear_start = probe_position_lf;

            // Can't re-enable (on error) until the new grid is written
            abl_should_enable = false;
        }

    } // !g29_in_progress

    {
        const ProbePtRaise raise_after = parser.boolval('E') ? PROBE_PT_STOW : PROBE_PT_RAISE;

        measured_z = 0;

        bool zig = PR_OUTER_END & 1; // Always end at RIGHT and BACK_PROBE_BED_POSITION

        measured_z = 0;

        xy_int8_t meshCount;

        // Outer loop is X with PROBE_Y_FIRST enabled
        // Outer loop is Y with PROBE_Y_FIRST disabled
        for (PR_OUTER_VAR = 0; PR_OUTER_VAR < PR_OUTER_END && !isnan(measured_z); PR_OUTER_VAR++) {

            int8_t inStart, inStop, inInc;

            if (zig) { // away from origin
                inStart = 0;
                inStop = PR_INNER_END;
                inInc = 1;
            } else { // towards origin
                inStart = PR_INNER_END - 1;
                inStop = -1;
                inInc = -1;
            }

            zig ^= true; // zag

            // An index to print current state
            uint8_t pt_index = (PR_OUTER_VAR) * (PR_INNER_END) + 1;

            // Inner loop is Y with PROBE_Y_FIRST enabled
            // Inner loop is X with PROBE_Y_FIRST disabled
            for (PR_INNER_VAR = inStart; PR_INNER_VAR != inStop; pt_index++, PR_INNER_VAR += inInc) {

                const xy_pos_t base = probe_position_lf.asFloat() + gridSpacing * meshCount.asFloat();

                probePos.set(FLOOR(base.x + (base.x < 0 ? 0 : 0.5)),
                    FLOOR(base.y + (base.y < 0 ? 0 : 0.5)));

#if IS_KINEMATIC
                // Avoid probing outside the round or hexagonal area
                if (!position_is_reachable_by_probe(probePos))
                    continue;
#endif

                if (verbose_level)
                    SERIAL_ECHOLNPAIR("Probing mesh point ", int(pt_index), "/", int(GRID_MAX_POINTS), ".");

                measured_z = faux ? 0.001 * random(-100, 101) : probe_at_point(probePos, raise_after, verbose_level);

                if (isnan(measured_z)) {
                    set_bed_leveling_enabled(abl_should_enable);
                    break; // Breaks out of both loops
                }

                z_values[meshCount.x][meshCount.y] = measured_z + zoffset;
#if ENABLED(EXTENSIBLE_UI)
                ExtUI::onMeshUpdate(meshCount.x, meshCount.y, z_values[meshCount.x][meshCount.y]);
#endif

                abl_should_enable = false;
                idle(false);

            } // inner
        }     // outer

        // Stow the probe. No raise for FIX_MOUNTED_PROBE.
        if (STOW_PROBE()) {
            set_bed_leveling_enabled(abl_should_enable);
            measured_z = NAN;
        }
    }

    //
    // G29 Finishing Code
    //
    // Unless this is a dry run, auto bed leveling will
    // definitely be enabled after this point.
    //
    // If code above wants to continue leveling, it should
    // return or loop before this point.
    //

    if (DEBUGGING(LEVELING))
        DEBUG_POS("> probing complete", current_position);

    // Calculate leveling, print reports, correct the position
    if (!isnan(measured_z)) {

        if (!dryrun)
            extrapolate_unprobed_bed_level();
        print_bilinear_leveling_grid();

        refresh_bed_level();

        if (!dryrun) {
            if (DEBUGGING(LEVELING))
                DEBUG_ECHOLNPAIR("G29 uncorrected Z:", current_position.z);

            // Unapply the offset because it is going to be immediately applied
            // and cause compensation movement in Z
            current_position.z -= bilinear_z_offset(current_position);

            if (DEBUGGING(LEVELING))
                DEBUG_ECHOLNPAIR(" corrected Z:", current_position.z);
        }

        // Auto Bed Leveling is complete! Enable if possible.
        planner.leveling_active = dryrun ? abl_should_enable : true;
    } // !isnan(measured_z)

    // Restore state after probing
    if (!faux)
        restore_feedrate_and_scaling();

    if (DEBUGGING(LEVELING))
        DEBUG_ECHOLNPGM("<<< G29");

    if (planner.leveling_active)
        sync_plan_position();

#if HAS_BED_PROBE && defined(Z_AFTER_PROBING)
    move_z_after_probing();
#endif

#ifdef Z_PROBE_END_SCRIPT
    if (DEBUGGING(LEVELING))
        DEBUG_ECHOLNPAIR("Z Probe End Script: ", Z_PROBE_END_SCRIPT);
    planner.synchronize();
    process_subcommands_now_P(PSTR(Z_PROBE_END_SCRIPT));
#endif

    report_current_position();
    G29_RETURN(isnan(measured_z));
    /// G29 end
}
