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
// from G29

std::array<xy_uint8_t, 9> skew_points = { { 14, 20 }, { 74, 20 }, { 146, 20 }, { 146, 89 }, { 74, 89 }, { 14, 99 }, { 14, 164 }, { 74, 164 }, { 146, 164 } };

xy_pos_t calculate_center(std::array<std::array<float, 32>, 32> z_grid) {
    return {};
}

static float run_z_probe(float min_z = -1) {
    // Probe downward slowly to find the bed
    if (do_probe_move(min_z, MMM_TO_MMS(Z_PROBE_SPEED_SLOW)))
        return NAN;
    return current_position.z;
}

float probe_at_skew_point(const xy_pos_t &pos) {
    xyz_pos_t npos = { pos.x, pos.y };
    if (!position_is_reachable_by_probe(npos))
        return NAN;       // The given position is in terms of the probe
    npos -= probe_offset; // Get the nozzle position

    npos.z = current_position.z;
    const float old_feedrate_mm_s = feedrate_mm_s;
    feedrate_mm_s = XY_PROBE_FEEDRATE_MM_S;

    // Move the probe to the starting XYZ
    do_blocking_move_to(npos);

    float measured_z = run_z_probe() + probe_offset.z;

    /// TODO: raise until untriggered
    do_blocking_move_to_z(current_position.z + (Z_CLEARANCE_BETWEEN_PROBES), MMM_TO_MMS(Z_PROBE_SPEED_FAST));

    feedrate_mm_s = old_feedrate_mm_s;
    if (isnan(measured_z))
        return 0;
    return measured_z;
}

void PrusaGcodeSuite::M45() {
    // TODO G28 if needed
    if (axis_unhomed_error())
        return;

    planner.synchronize();
    set_bed_leveling_enabled(false);
    remember_feedrate_scaling_off();

    xy_pos_t probePos;
    xy_probe_feedrate_mm_s = MMM_TO_MMS(XY_PROBE_SPEED);
    float measured_z = 0;
    std::array<std::array<float, 32>, 32> z_grid;
    std::array<std::array<xy_pos_t, 3>, 3> centers;

    /// cycle over 9 points
    for (int8_t py = 0; py < 3; ++py) {
        for (int8_t px = 0; px < 3; ++px) {
            probePos = skew_points[3 * py + px];
            /// TODO: find safe Z

            /// scan 32x32 array
            for (int8_t y = 0; y < 32; ++y) {
                for (int8_t x = 0; x < 32; ++x) {
                    probePos.x += x - 32 / 2 + .5f;
                    probePos.y += y - 32 / 2 + .5f;
                    /// FIXME: don't go too low
                    measured_z = probe_at_skew_point(probePos);
                    z_grid[x][y] = isnan(measured_z) ? -100.f : measured_z;
                    idle(false);
                }
            }

            /// print point grid
            // print_2d_array(z_grid.size(), z_grid[0].size(), 3, [](const uint8_t ix, const uint8_t iy) { return z_grid[ix][iy]; });

            centers[px][py] = calculate_center(z_grid);
        }
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
