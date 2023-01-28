#include "PrusaGcodeSuite.hpp"

// from G29
#include "../../lib/Marlin/Marlin/src/inc/MarlinConfig.h"
#include "../../lib/Marlin/Marlin/src/gcode/gcode.h"
#include "../../lib/Marlin/Marlin/src/feature/bedlevel/bedlevel.h"
#include "../../lib/Marlin/Marlin/src/module/motion.h"
#include "../../lib/Marlin/Marlin/src/module/planner.h"
#include "../../lib/Marlin/Marlin/src/module/stepper.h"
#include "../../lib/Marlin/Marlin/src/module/probe.h"
#include "../../lib/Marlin/Marlin/src/gcode/queue.h"

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

#define G29_RETURN(b) return;

// end from G29

xy_pos_t calculate_center(std::array<std::array<float, 32>, 32> z_grid) {
    return {};
}

static float run_z_probe() {

    // Stop the probe before it goes too low to prevent damage.
    // If Z isn't known then probe to -10mm.
    const float z_probe_low_point = TEST(axis_known_position, Z_AXIS) ? -probe_offset.z + Z_PROBE_LOW_POINT : -10.0;

    {
        // Probe downward slowly to find the bed
        if (do_probe_move(z_probe_low_point, MMM_TO_MMS(Z_PROBE_SPEED_SLOW))) {
            if (planner.draining())
                return NAN;

            if (DEBUGGING(LEVELING)) {
                DEBUG_ECHOLNPGM("SLOW Probe fail!");
                DEBUG_POS("<<< run_z_probe", current_position);
            }
            return NAN;
        }

#if ENABLED(MEASURE_BACKLASH_WHEN_PROBING)
        backlash.measure_with_probe();
#endif

        const float z = current_position.z;

#if EXTRA_PROBING
        // Insert Z measurement into probes[]. Keep it sorted ascending.
        for (uint8_t i = 0; i <= p; i++) { // Iterate the saved Zs to insert the new Z
            if (i == p || probes[i] > z) { // Last index or new Z is smaller than this Z
                for (int8_t m = p; --m >= i;)
                    probes[m + 1] = probes[m]; // Shift items down after the insertion point
                probes[i] = z;                 // Insert the new Z measurement
                break;                         // Only one to insert. Done!
            }
        }
#elif TOTAL_PROBING > 2
        probes_total += z;
#else
        UNUSED(z);
#endif

#if TOTAL_PROBING > 2
        // Small Z raise after all but the last probe
        if (p
    #if EXTRA_PROBING
            < TOTAL_PROBING - 1
    #endif
        )
            do_blocking_move_to_z(z + Z_CLEARANCE_MULTI_PROBE, MMM_TO_MMS(Z_PROBE_SPEED_FAST));
#endif
    }

#if TOTAL_PROBING > 2

    #if EXTRA_PROBING
    // Take the center value (or average the two middle values) as the median
    static constexpr int PHALF = (TOTAL_PROBING - 1) / 2;
    const float middle = probes[PHALF],
                median = ((TOTAL_PROBING)&1) ? middle : (middle + probes[PHALF + 1]) * 0.5f;

    // Remove values farthest from the median
    uint8_t min_avg_idx = 0, max_avg_idx = TOTAL_PROBING - 1;
    for (uint8_t i = EXTRA_PROBING; i--;)
        if (ABS(probes[max_avg_idx] - median) > ABS(probes[min_avg_idx] - median))
            max_avg_idx--;
        else
            min_avg_idx++;

    // Return the average value of all remaining probes.
    for (uint8_t i = min_avg_idx; i <= max_avg_idx; i++)
        probes_total += probes[i];

    #endif

    const float measured_z = probes_total * RECIPROCAL(MULTIPLE_PROBING);

#elif TOTAL_PROBING == 2

    const float z2 = current_position.z;

    if (DEBUGGING(LEVELING))
        DEBUG_ECHOLNPAIR("2nd Probe Z:", z2, " Discrepancy:", first_probe_z - z2);

    // Return a weighted average of the fast and slow probes
    const float measured_z = (z2 * 3.0 + first_probe_z * 2.0) * 0.2;

#else

    // Return the single probe result
    const float measured_z = current_position.z;

#endif

    if (DEBUGGING(LEVELING))
        DEBUG_POS("<<< run_z_probe", current_position);

    return measured_z;
}

float probe_at_skew_point(const xy_pos_t &pos, const ProbePtRaise raise_after = PROBE_PT_NONE) {
    xyz_pos_t npos = { pos.x, pos.y };
    if (!position_is_reachable_by_probe(npos))
        return NAN;       // The given position is in terms of the probe
    npos -= probe_offset; // Get the nozzle position

    npos.z = current_position.z;
    const float old_feedrate_mm_s = feedrate_mm_s;
    feedrate_mm_s = XY_PROBE_FEEDRATE_MM_S;

    // Move the probe to the starting XYZ
    do_blocking_move_to(npos);

    float measured_z = NAN;
    if (!DEPLOY_PROBE()) {
        measured_z = run_z_probe() + probe_offset.z;

        const bool big_raise = raise_after == PROBE_PT_BIG_RAISE;
        if (big_raise || raise_after == PROBE_PT_RAISE)
            do_blocking_move_to_z(current_position.z + (big_raise ? 25 : Z_CLEARANCE_BETWEEN_PROBES), MMM_TO_MMS(Z_PROBE_SPEED_FAST));
        else if (raise_after == PROBE_PT_STOW)
            if (STOW_PROBE())
                measured_z = NAN;
    }

    feedrate_mm_s = old_feedrate_mm_s;
    if (isnan(measured_z)) {
        STOW_PROBE();
        LCD_MESSAGEPGM(MSG_LCD_PROBING_FAILED);
        SERIAL_ERROR_MSG(MSG_ERR_PROBING_FAILED);
    }
    return measured_z;
}

void PrusaGcodeSuite::M45() {
    // TODO G28 if needed
    if (axis_unhomed_error())
        G29_RETURN(false);

    // Define local vars 'static' for manual probing, 'auto' otherwise

    xy_pos_t probePos;
    xy_int_t probe_position_lf, probe_position_rb;
    xy_float_t gridSpacing = { 0, 0 };
    constexpr xy_uint8_t abl_grid_points = { GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y };

    /**
     * On the initial G29 fetch command parameters.
     */

    bool abl_should_enable = planner.leveling_active;
    xy_probe_feedrate_mm_s = MMM_TO_MMS(XY_PROBE_SPEED);

    const float x_min = probe_min_x(), x_max = probe_max_x(),
                y_min = probe_min_y(), y_max = probe_max_y();

    probe_position_lf.set(
        _MAX(X_CENTER - (X_BED_SIZE) / 2, x_min),
        _MAX(Y_CENTER - (Y_BED_SIZE) / 2, y_min));
    probe_position_rb.set(
        _MIN(probe_position_lf.x + X_BED_SIZE, x_max),
        _MIN(probe_position_lf.y + Y_BED_SIZE, y_max));

    // probe at the points of a lattice grid
    gridSpacing.set((probe_position_rb.x - probe_position_lf.x) / (abl_grid_points.x - 1),
        (probe_position_rb.y - probe_position_lf.y) / (abl_grid_points.y - 1));

    planner.synchronize();

    // Disable auto bed leveling during G29.
    // Be formal so G29 can be done successively without G28.
    set_bed_leveling_enabled(false);

    // Deploy the probe. Probe will raise if needed.
    if (DEPLOY_PROBE()) {
        set_bed_leveling_enabled(abl_should_enable);
        G29_RETURN(false);
    }

    remember_feedrate_scaling_off();

    const ProbePtRaise raise_after = PROBE_PT_RAISE;
    float measured_z = 0;
    xy_int8_t meshCount;
    std::array<std::array<float, 32>, 32> z_grid;
    std::array<std::array<xy_pos_t, 3>, 3> centers;

    /// cycle over 9 points
    for (int8_t py = 0; py < 3; ++py) {
        for (int8_t px = 0; px < 3; ++px) {

            const xy_pos_t base = probe_position_lf.asFloat() + gridSpacing * meshCount.asFloat();
            probePos.set(FLOOR(base.x + (base.x < 0 ? 0 : 0.5f)),
                FLOOR(base.y + (base.y < 0 ? 0 : 0.5f)));

            /// find safe Z

            /// scan 32x32 array
            for (int8_t y = 0; y < 32; ++y) {
                for (int8_t x = 0; x < 32; ++x) {
                    probePos.x += x - 32 / 2 + .5f;
                    probePos.y += y - 32 / 2 + .5f;
                    /// FIXME: don't go too low
                    measured_z = probe_at_skew_point(probePos, raise_after);
                    z_grid[x][y] = isnan(measured_z) ? -100.f : measured_z;
                    idle(false);
                }
            }

            /// print point grid
            // print_2d_array(z_grid.size(), z_grid[0].size(), 3, [](const uint8_t ix, const uint8_t iy) { return z_grid[ix][iy]; });

            centers[px][py] = calculate_center(z_grid);
        }
    }

    // Stow the probe. No raise for FIX_MOUNTED_PROBE.
    if (STOW_PROBE()) {
        set_bed_leveling_enabled(true);
        measured_z = NAN;
    }

    /// TODO: print centers

    current_position.z -= bilinear_z_offset(current_position);
    planner.leveling_active = true;
    restore_feedrate_and_scaling();
    if (planner.leveling_active)
        sync_plan_position();
    idle(false);
    /// TODO: convert to non-blocking move
    move_z_after_probing();

    /// calculate XY skews
    /// pick median
    /// set XY skew

    planner.synchronize();
    report_current_position();
}
