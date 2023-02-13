#include "PrusaGcodeSuite.hpp"

#include "../../lib/Marlin/Marlin/src/inc/MarlinConfig.h"
#include "../../lib/Marlin/Marlin/src/gcode/gcode.h"
#include "../../lib/Marlin/Marlin/src/feature/bedlevel/bedlevel.h"
#include "../../lib/Marlin/Marlin/src/module/motion.h"
#include "../../lib/Marlin/Marlin/src/module/planner.h"
#include "../../lib/Marlin/Marlin/src/module/stepper.h"
#include "../../lib/Marlin/Marlin/src/module/probe.h"
#include "../../lib/Marlin/Marlin/src/gcode/queue.h"
#include "../../lib/Marlin/Marlin/src/gcode/motion/G2_G3.h"
#include "../../lib/Marlin/Marlin/src/module/endstops.h"

const float radius_8 = 3.f;

xy_pos_t get_skew_point(int8_t ix, int8_t iy) {
    // std::array<xy_uint8_t, 9> skew_points = { { 14, 20 }, { 74, 20 }, { 146, 20 }, { 146, 89 }, { 74, 89 }, { 14, 99 }, { 14, 164 }, { 74, 164 }, { 146, 164 } };
    const xyz_pos_t probe = NOZZLE_TO_PROBE_OFFSET;

    if (ix == 0 && iy == 1)
        return xy_pos_t { 14 - probe.x, 99 - probe.y };

    xy_pos_t pos;
    switch (ix) {
    case 0:
        pos.x = 14 - probe.x;
        break;
    case 1:
        pos.x = 74 - probe.x;
        break;
    case 2:
        pos.x = 146 - probe.x;
        break;
    }

    switch (iy) {
    case 0:
        pos.y = 20 - probe.y;
        break;
    case 1:
        pos.y = 89 - probe.y;
        break;
    case 2:
        pos.y = 164 - probe.y;
        break;
    }
    return pos;
}

void print_area(std::array<std::array<float, 32>, 32> z_grid) {
    SERIAL_EOL();
    for (int8_t y = 0; y < 32; ++y) {
        for (int8_t x = 0; x < 32; ++x) {
            SERIAL_ECHO(z_grid[x][y]);
            SERIAL_CHAR(' ');
        }
        SERIAL_EOL();
    }
    SERIAL_EOL();
}

xy_pos_t calculate_center(std::array<std::array<float, 32>, 32> z_grid) {
    return {};
}

void print_centers(std::array<std::array<xy_pos_t, 3>, 3> centers) {
    SERIAL_EOL();
    for (int8_t y = 0; y < 3; ++y) {
        for (int8_t x = 0; x < 3; ++x) {
            SERIAL_CHAR('[');
            SERIAL_ECHO(centers[x][y].x);
            SERIAL_CHAR(',');
            SERIAL_ECHO(centers[x][y].y);
            SERIAL_CHAR('[');
            SERIAL_CHAR(' ');
        }
        SERIAL_EOL();
    }
    SERIAL_EOL();
}

void calculate_skews(std::array<std::array<xy_pos_t, 3>, 3> centers) {
}

static float run_z_probe(float min_z = -1) {
    // Probe downward slowly to find the bed
    if (do_probe_move(min_z, MMM_TO_MMS(Z_PROBE_SPEED_SLOW)))
        return NAN;
    return current_position.z;
}

float probe_at_skew_point(const xy_pos_t &pos) {
    xyz_pos_t npos = { pos.x, pos.y };
    npos.z = current_position.z;
    if (!position_is_reachable_by_probe(npos))
        return NAN; // The given position is in terms of the probe

    const float old_feedrate_mm_s = feedrate_mm_s;
    feedrate_mm_s = XY_PROBE_FEEDRATE_MM_S;

    // Move the probe to the starting XYZ
    do_blocking_move_to(npos);

    float measured_z = run_z_probe();

    /// TODO: raise until untriggered
    do_blocking_move_to_z(npos.z + (Z_CLEARANCE_BETWEEN_PROBES), MMM_TO_MMS(Z_PROBE_SPEED_FAST));

    feedrate_mm_s = old_feedrate_mm_s;
    if (isnan(measured_z))
        return 0;
    return measured_z;
}

// Do "8" moves and stop if probe triggered or too low.
// \returns true if probe triggered
bool find_safe_z() {
    bool hit = false;

// Disable stealthChop if used. Enable diag1 pin on driver.
#if ENABLED(SENSORLESS_PROBING)
    sensorless_t stealth_states { false };
    #if ENABLED(DELTA)
    stealth_states.x = tmc_enable_stallguard(stepperX);
    stealth_states.y = tmc_enable_stallguard(stepperY);
    #endif
    stealth_states.z = tmc_enable_stallguard(stepperZ);
    endstops.enable(true);
#endif

    float z = current_position.z;
    for (int step = 0; z > 0; ++step) {
        if (step % 2) {
            // CCW circle
            plan_arc(current_position, ab_float_t { 0.f, -radius_8 }, false);
        } else {
            // CW circle
            plan_arc(current_position, ab_float_t { 0.f, radius_8 }, true);
        }

        // Check to see if the probe was triggered
        hit = TEST(endstops.trigger_state(), Z_MIN);
        if (hit)
            break;

        z -= .3f;
        do_blocking_move_to_z(z, MMM_TO_MMS(Z_PROBE_SPEED_FAST));
    }

// Re-enable stealthChop if used. Disable diag1 pin on driver.
#if ENABLED(SENSORLESS_PROBING)
    endstops.not_homing();
    #if ENABLED(DELTA)
    tmc_disable_stallguard(stepperX, stealth_states.x);
    tmc_disable_stallguard(stepperY, stealth_states.y);
    #endif
    tmc_disable_stallguard(stepperZ, stealth_states.z);
#endif

    // Clear endstop flags
    endstops.hit_on_purpose();
    // Get Z where the steppers were interrupted
    set_current_from_steppers_for_axis(Z_AXIS);
    // Tell the planner where we actually are
    sync_plan_position();
    return hit;
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
            probePos = get_skew_point(px, py);
            SERIAL_ECHO(current_position.z);
            SERIAL_EOL();
            do_blocking_move_to_z(2, MMM_TO_MMS(Z_PROBE_SPEED_FAST));
            SERIAL_ECHO(current_position.z);
            SERIAL_EOL();
            do_blocking_move_to(probePos, xy_probe_feedrate_mm_s);
            SERIAL_ECHO(current_position.z);
            SERIAL_EOL();
            bool is_z = find_safe_z();
            SERIAL_ECHO(current_position.z);
            SERIAL_EOL();

            if (!is_z) {
                centers[px][py] = xy_pos_t { NAN, NAN };
                continue;
            }

            // /// scan 32x32 array
            // for (int8_t y = 0; y < 32; ++y) {
            //     for (int8_t x = 0; x < 32; ++x) {
            //         probePos.x += x - 32 / 2 + .5f;
            //         probePos.y += y - 32 / 2 + .5f;
            //         /// FIXME: don't go too low
            //         measured_z = probe_at_skew_point(probePos);
            //         z_grid[x][y] = isnan(measured_z) ? -100.f : measured_z;
            //         idle(false);
            //     }
            // }

            // /// print point grid
            // // print_2d_array(z_grid.size(), z_grid[0].size(), 3, [](const uint8_t ix, const uint8_t iy) { return z_grid[ix][iy]; });

            // print_area(z_grid);

            // //                 SERIAL_ECHO(int(x));
            // //   SERIAL_EOL();
            // //   SERIAL_ECHOLNPGM("measured_z = ["); // open 2D array

            // centers[px][py] = calculate_center(z_grid);
        }
    }

    // print_centers(centers);

    current_position.z -= bilinear_z_offset(current_position);
    planner.leveling_active = true;
    restore_feedrate_and_scaling();
    if (planner.leveling_active)
        sync_plan_position();
    idle(false);
    /// TODO: convert to non-blocking move
    move_z_after_probing();

    // calculate_skews(centers);
    /// pick median
    /// set XY skew

    planner.synchronize();
    report_current_position();
}

/// Skew computation:

/// M  ... measured points (points in columns) (XY)
/// I  ... ideal point position matrix (points in columns) (XY)
/// SH1... 1st shift matrix (2 variables)
/// R  ... rotation matrix (1 variable)
/// SH2... 2nd shift matrix (2 variables)
/// SK ... skew matrix (1 variable)

/// SK =  1 sk
///       0  1

/// SK * (SH2 + R * (SH1 + I)) = M

/// sh2x + co * (sh1x + ix) - si * (sh1y + iy) + sk * my  = mx
/// sh2y + si * (sh1x + ix) + co * (sh1y + iy)            = my
/// co = cos(phi), si = sin(phi)

/// 3 (XY) points needed to fully define all 6 variables.
/// Too complicated. But distance (of 2 points) is not dependent on shift or rotation.
/// Still, we need rotation to apply proper skew.

/// B  ... base vectors of points (shifted to [0,0]) (XY)

/// ||SK * R * B|| = ||M||

/// px = co * bx - si * by   // x part
/// py = si * bx + co * by   // y part
/// ||px + sk * py, py|| = ||mx, my||

/// Ax + b = 0
///  ||px1 + sk * py1, py1|| - ||mx1, my1|| = 0
///  ||px2 + sk * py2, py2|| - ||mx2, my2|| = 0

/// Now, 2 vectors (3 or 4 points) provide over-defined equation which eliminates some error.
/// Ideally, vectors should not be in similar direction for precise results (right angle is the best).
/// Error plane should have at most 2 local minimas which is ideal for iterative numerical method.
/// We can set rotation limits to +/- 20° because it will be, typically, only few degrees.
/// Use golden ratio 0.618.
/// Guess rotation and compute skew coefficient. Minimize error.

/// err = (px + sk * py)^2 + py^2 - (mx^2 + my^2)
/// err = px^2 + sk^2 * py^2 + 2 * px * sk * py + py^2 - mx^2 - my^2

/// err = sk^2 * (py1^2) + sk * (2 * px1 * py1) + (py1^2 + px1^2 - mx1^2 - my1^2)
/// err = sk^2 * (py2^2) + sk * (2 * px2 * py2) + (py2^2 + px2^2 - mx2^2 - my2^2)
