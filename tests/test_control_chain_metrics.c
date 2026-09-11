#include "tests/test_framework.h"
#include "hal/hal_host.h"
#include "estimation/estimator.h"
#include "guidance/v_safe.h"
#include "guidance/pd_controller.h"
#include "guidance/actuator_mapping.h"
#include "common/metrics.h"
#include <math.h>

#define TICK_DT 0.01
#define N_TICKS 500

/* Real 1m/1m rig envelope (not widened) — same recalibration principle as
   the task-15 fix: sim_cart_step() clamps true_range_m to
   [0, track_length_m] but never clamps true_velocity_mps, and the
   true_accel_mps2 fed to hal_host_world_tick() here is a scripted
   "hand-push" input, not fed back from the PD controller (the cart is
   hand-pushed and the servo has no authority over its motion — see
   guidance/actuator_mapping.h, docs/design.md §7.3/§9), so this scenario
   is open-loop with respect to the plant exactly like task 15's was. The
   brief's original constant accel of 0.2 m/s^2 for the full 5s run would
   drive true_velocity_mps to 1.0 m/s (67% over v_cap=0.6 m/s) and would
   run the cart's true_range_m into its 0 clamp at ~t=3.16s (tick ~316 of
   500), silently diverging ground truth from anything observable for the
   remaining ~184 ticks. Recalibrated to 0.05 m/s^2: over the full 5s run,
   final true_velocity_mps = 0.25 m/s (comfortably under v_cap=0.6) and
   total travel = 0.625 m, leaving true_range_m = 0.375 m at the end —
   the range clamp is never hit and ground truth stays valid for all
   500 ticks. */
#define TRUE_ACCEL_MPS2 0.05

TEST(test_rms_tracking_error_and_saturation_percentage) {
    hal_host_world_t w;
    hal_host_world_init(&w, 1.0, 1.0, 11);
    estimator_t e;
    estimator_init(&e, 0.90);
    pd_controller_t pd;
    pd_controller_init(&pd, 1.5, 0.1, 0.05);

    metrics_accum_t tracking_m, control_output_m;
    metrics_init(&tracking_m);
    metrics_init(&control_output_m);
    int saturated_ticks = 0;

    /* Last known MEASURED range, as a real onboard controller would hold
       it between ~20Hz ToF samples (see sim/sim_range_sensor.c's sample
       period) — never ground truth. Seeded from the same initial_range_m
       used to construct the simulated world (a legitimate startup
       condition), then updated only on HAL_OK reads and held otherwise. */
    double measured_range_m = 1.0; /* matches hal_host_world_init's initial_range_m below */

    for (int i = 0; i < N_TICKS; i++) {
        hal_host_world_tick(&w, TRUE_ACCEL_MPS2, 0.0, TICK_DT);
        hal_t h = hal_host_create(&w);
        range_sample_t r; imu_sample_t s;
        hal_status_t rs = h.range_read(h.ctx, &r);
        hal_status_t is = h.imu_read(h.ctx, &s);
        r.status = rs;
        s.status = is;
        if (rs == HAL_OK) measured_range_m = r.range_m;
        estimator_output_t est = estimator_tick(&e, &r, &s);

        double target = v_safe(measured_range_m);
        double speed_error = est.fused_speed_mps - target;
        pd_output_t ctrl = pd_controller_update(&pd, speed_error, TICK_DT);
        double servo_deg = actuator_map_to_servo_deg(ctrl.control_output_filtered);

        metrics_add(&tracking_m, est.fused_speed_mps, target);
        metrics_add(&control_output_m, ctrl.control_output_filtered, 0.0);
        if (fabs(ctrl.control_output_filtered) >= 0.999) saturated_ticks++;
        CHECK(servo_deg >= 45.0 && servo_deg <= 135.0);
    }

    double saturation_pct = 100.0 * saturated_ticks / N_TICKS;
    CHECK(metrics_rmse(&tracking_m) < 0.5);
    CHECK(saturation_pct >= 0.0 && saturation_pct <= 100.0);
}

TEST(test_derivative_jitter_reduced_by_filtering_on_noisy_run) {
    hal_host_world_t w;
    hal_host_world_init(&w, 1.0, 1.0, 13);
    estimator_t e;
    estimator_init(&e, 0.90);
    pd_controller_t pd;
    pd_controller_init(&pd, 1.5, 0.5, 0.05); /* higher Kd to make jitter visible */

    jitter_accum_t j_unfiltered, j_filtered;
    jitter_init(&j_unfiltered);
    jitter_init(&j_filtered);

    /* Same held/measured-range discipline as test 1 above — never ground
       truth, seeded from this world's initial_range_m. */
    double measured_range_m = 1.0; /* matches hal_host_world_init's initial_range_m below */

    for (int i = 0; i < N_TICKS; i++) {
        hal_host_world_tick(&w, TRUE_ACCEL_MPS2, 0.0, TICK_DT);
        hal_t h = hal_host_create(&w);
        range_sample_t r; imu_sample_t s;
        hal_status_t rs = h.range_read(h.ctx, &r);
        hal_status_t is = h.imu_read(h.ctx, &s);
        r.status = rs;
        s.status = is;
        if (rs == HAL_OK) measured_range_m = r.range_m;
        estimator_output_t est = estimator_tick(&e, &r, &s);
        double speed_error = est.fused_speed_mps - v_safe(measured_range_m);
        pd_output_t ctrl = pd_controller_update(&pd, speed_error, TICK_DT);
        jitter_add(&j_unfiltered, ctrl.control_output_unfiltered);
        jitter_add(&j_filtered, ctrl.control_output_filtered);
    }
    /* This is the demonstrable point: unfiltered D-term jitters more than
       filtered, on the same run (docs/design.md §7.2, §8). */
    CHECK(jitter_rms(&j_unfiltered) > jitter_rms(&j_filtered));
}

TEST(test_step_response_time_to_speed_error_step) {
    pd_controller_t pd;
    pd_controller_init(&pd, 1.5, 0.1, 0.05);
    double t_s[N_TICKS]; double output[N_TICKS];
    for (int i = 0; i < N_TICKS; i++) {
        double speed_error = (i < 100) ? 0.0 : 0.3; /* step in error at t=1.0s */
        pd_output_t ctrl = pd_controller_update(&pd, speed_error, TICK_DT);
        t_s[i] = i * TICK_DT;
        output[i] = ctrl.control_output_filtered;
    }
    double final_output = output[N_TICKS - 1];
    double settle = find_settling_time(t_s, output, N_TICKS, final_output, 0.05);
    CHECK(settle >= 1.0); /* can't settle before the step happens */
    CHECK(settle < t_s[N_TICKS - 1]);
}

int main(void) {
    RUN_TEST(test_rms_tracking_error_and_saturation_percentage);
    RUN_TEST(test_derivative_jitter_reduced_by_filtering_on_noisy_run);
    RUN_TEST(test_step_response_time_to_speed_error_step);
    TEST_SUMMARY();
    return 0;
}
