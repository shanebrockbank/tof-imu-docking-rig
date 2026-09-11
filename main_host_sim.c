#include <stdio.h>
#include "hal/hal_host.h"
#include "estimation/estimator.h"
#include "guidance/v_safe.h"
#include "guidance/pd_controller.h"
#include "guidance/actuator_mapping.h"
#include "logging/csv_logger.h"
#include "common/dt_validation.h"

#define TICK_DT 0.01
#define N_TICKS 1000 /* 10 seconds */
#define TRACK_LENGTH_M 1.0
#define INITIAL_RANGE_M 1.0

/* Scripted hand-push acceleration profile (ground truth for the demo cart).
 *
 * sim_cart_step() clamps true_range_m to [0, track_length_m] but NEVER
 * clamps true_velocity_mps, so an open-loop accel profile that is too
 * large/long for the 1.0 m track runs the cart into the wall (range pinned
 * at 0) while velocity keeps climbing unchecked, silently diverging ground
 * truth from anything observable for the rest of the run.
 *
 * The plan-brief's original two-phase profile (+0.3 m/s^2 for the first
 * 3 s, then 0 m/s^2) does exactly that: range(t) = 1.0 - 0.15*t^2 hits 0 at
 * t = sqrt(1.0/0.15) = 2.582 s -- i.e. the cart hits the wall about 0.4 s
 * *before* the scripted acceleration phase even ends, at which point
 * v = 0.3*2.582 = 0.775 m/s and still rising toward 0.9 m/s at t=3s. That
 * both blows the 1.0 m track (before it's even clamped) and wildly
 * exceeds v_cap = 0.6 m/s. It was replaced with a four-phase
 * accelerate/coast/decelerate/hold trapezoid, recalibrated by hand so the
 * cart performs a real docking approach that stays inside the track with
 * margin and stays under v_cap throughout:
 *
 *   Phase A  [0.0s, 2.0s)  accel = +0.15 m/s^2  (ticks   0..199)
 *   Phase B  [2.0s, 3.0s)  accel =  0.0 m/s^2   (ticks 200..299)  coast
 *   Phase C  [3.0s, 5.0s)  accel = -0.15 m/s^2  (ticks 300..499)  decel
 *   Phase D  [5.0s,10.0s)  accel =  0.0 m/s^2   (ticks 500..999)  hold
 *
 * Hand-computed trajectory (v0 = 0, range0 = 1.0 m):
 *   t=2.0s: v = 0.15*2.0        = 0.30 m/s   distance = 0.075*2.0^2       = 0.30 m  -> range = 0.70 m
 *   t=3.0s: v const at 0.30 m/s              distance += 0.30*1.0        = 0.30 m  -> range = 0.40 m
 *   t=5.0s: v = 0.30 - 0.15*2.0 = 0.00 m/s   distance += 0.30*2.0/2      = 0.30 m  -> range = 0.10 m
 *   t=10.0s: accel=0, v holds at ~0.00 m/s, range holds at ~0.10 m
 *
 * Totals: distance traveled = 0.90 m (< 1.0 m track, 0.10 m margin, so the
 * [0, track_length_m] clamp in sim_cart_step never actually engages), peak
 * velocity = 0.30 m/s (well under v_cap = 0.6 m/s), and the cart coasts to
 * a controlled near-stop rather than crashing. This also gives the PD
 * controller something interesting to react to: v_safe(range) drops below
 * the cart's 0.30 m/s cruise speed once range < (0.30/0.5)^2 = 0.36 m
 * (partway through Phase B/C), producing a real, growing speed_error for
 * the guidance/control chain to respond to as the approach tightens.
 */
#define PUSH_ACCEL_MPS2 0.15
#define PHASE_A_END_TICK 200 /* t < 2.0s: accelerate */
#define PHASE_B_END_TICK 300 /* 2.0s <= t < 3.0s: coast */
#define PHASE_C_END_TICK 500 /* 3.0s <= t < 5.0s: decelerate */
/* t >= 5.0s: hold (accel = 0) */

int main(void) {
    hal_host_world_t world;
    hal_host_world_init(&world, INITIAL_RANGE_M, TRACK_LENGTH_M, 1234);

    estimator_t estimator;
    estimator_init(&estimator, 0.90);

    pd_controller_t pd;
    pd_controller_init(&pd, 1.5, 0.1, 0.05);

    csv_logger_t log;
    if (!csv_logger_open(&log, "run_log.csv")) {
        fprintf(stderr, "failed to open run_log.csv for writing\n");
        return 1;
    }

    /* hal_t is populated once at startup (docs/design.md §5), not
     * reconstructed every loop iteration -- it's just a fixed set of
     * function pointers bound to `world`. */
    hal_t h = hal_host_create(&world);
    /* Previous tick's timestamp, for computing the PD controller's dt from
     * real clock_now() readings rather than assuming a fixed TICK_DT. */
    timestamp_t prev_now = h.clock_now(h.ctx);

    /* Held last-known MEASURED range, for driving v_safe(). Ground truth
     * (world.cart.true_range_m) is used below only for the CSV's reference
     * columns -- never as a computational input to v_safe()/the PD
     * controller, since a real vehicle only ever has the measured range.
     * The ToF sensor is ~20 Hz against a 100 Hz main loop, so range_read()
     * returns HAL_NO_NEW_DATA most ticks; we hold the last fresh reading
     * across those ticks rather than reading world.cart.true_range_m or an
     * uninitialized sample buffer. */
    double last_measured_range_m = INITIAL_RANGE_M;

    for (int i = 0; i < N_TICKS; i++) {
        double true_accel_mps2;
        if (i < PHASE_A_END_TICK) {
            true_accel_mps2 = PUSH_ACCEL_MPS2;
        } else if (i < PHASE_B_END_TICK) {
            true_accel_mps2 = 0.0;
        } else if (i < PHASE_C_END_TICK) {
            true_accel_mps2 = -PUSH_ACCEL_MPS2;
        } else {
            true_accel_mps2 = 0.0;
        }

        hal_host_world_tick(&world, true_accel_mps2, 0.0, TICK_DT);
        timestamp_t now = h.clock_now(h.ctx);
        double ctrl_dt = dt_between(prev_now, now);

        range_sample_t range_sample;
        imu_sample_t imu_sample;
        hal_status_t range_status = h.range_read(h.ctx, &range_sample);
        hal_status_t imu_status = h.imu_read(h.ctx, &imu_sample);
        range_sample.status = range_status;
        imu_sample.status = imu_status;

        if (range_status == HAL_OK) {
            last_measured_range_m = range_sample.range_m;
        }

        estimator_output_t est = estimator_tick(&estimator, &range_sample, &imu_sample);

        double target = v_safe(last_measured_range_m);
        double speed_error = est.fused_speed_mps - target;
        pd_output_t ctrl = pd_controller_update(&pd, speed_error, ctrl_dt);
        double servo_deg = actuator_map_to_servo_deg(ctrl.control_output_filtered);
        h.actuator_set_angle_deg(h.ctx, servo_deg);

        csv_logger_write_row(&log,
            now.t_s,
            world.cart.true_range_m,
            world.cart.true_velocity_mps,
            est.raw_speed.value_mps,
            est.raw_speed.is_stale,
            est.fused_speed_mps,
            target,
            speed_error,
            ctrl.control_output_unfiltered,
            ctrl.control_output_filtered,
            servo_deg);

        prev_now = now;
    }

    csv_logger_close(&log);
    printf("wrote run_log.csv (%d ticks, %.1fs)\n", N_TICKS, N_TICKS * TICK_DT);
    return 0;
}
