#include "tests/test_framework.h"
#include "hal/hal_host.h"
#include "estimation/estimator.h"
#include "common/metrics.h"
#include <stdio.h>

#define TICK_DT 0.01
#define N_TICKS 500 /* 5 seconds */

TEST(test_fused_rmse_bias_variance_under_nominal_noise) {
    hal_host_world_t w;
    /* Real 1m/1m rig envelope (not widened). sim_cart_step() clamps
       true_range_m to [0, track_length_m] but never clamps
       true_velocity_mps, so the acceleration below is calibrated to keep
       total travel and final velocity comfortably inside the 1m track /
       v_cap=0.6 m/s envelope for the full 5s window (see recalibration
       note in the task-15 fix commit). */
    hal_host_world_init(&w, 1.0, 1.0, 42);
    estimator_t e;
    estimator_init(&e, 0.90);

    metrics_accum_t raw_m, fused_m;
    metrics_init(&raw_m);
    metrics_init(&fused_m);

    double true_accel = 0.05; /* constant gentle push toward target */
    for (int i = 0; i < N_TICKS; i++) {
        hal_host_world_tick(&w, true_accel, 0.0, TICK_DT);
        hal_t h = hal_host_create(&w);
        range_sample_t r; imu_sample_t s;
        hal_status_t rs = h.range_read(h.ctx, &r);
        hal_status_t is = h.imu_read(h.ctx, &s);
        r.status = rs; s.status = is;
        estimator_output_t out = estimator_tick(&e, &r, &s);

        double true_v = w.cart.true_velocity_mps; /* ground truth, never fed to estimator_tick */
        if (!out.raw_speed.is_stale) metrics_add(&raw_m, out.raw_speed.value_mps, true_v);
        metrics_add(&fused_m, out.fused_speed_mps, true_v);
    }

    /* FUSED must track ground truth substantially better than RAW's raw
       differencing noise floor (the demonstrable point of fusion). */
    printf("RAW RMSE = %.4f m/s\n", metrics_rmse(&raw_m));
    printf("FUSED RMSE = %.4f m/s\n", metrics_rmse(&fused_m));
    printf("FUSED bias = %.4f m/s\n", metrics_mean_bias(&fused_m));
    printf("FUSED variance = %.6f\n", metrics_variance(&fused_m));
    CHECK(metrics_rmse(&fused_m) < metrics_rmse(&raw_m));
    CHECK(metrics_rmse(&fused_m) < 0.15);
    CHECK(fabs(metrics_mean_bias(&fused_m)) < 0.1);
}

TEST(test_dropout_recovery_fused_reconverges_after_tof_gap) {
    hal_host_world_t w;
    hal_host_world_init(&w, 1.0, 1.0, 7); /* real 1m/1m rig envelope, see note above */
    estimator_t e;
    estimator_init(&e, 0.90);

    double true_accel = 0.04;
    double t_s[N_TICKS]; double fused[N_TICKS];
    for (int i = 0; i < N_TICKS; i++) {
        bool in_gap = (i >= 100 && i < 130); /* ~300ms ToF gap, well under DT_MAX_S=2.0 */
        hal_host_world_tick(&w, true_accel, 0.0, TICK_DT);
        if (in_gap) w.range_sensor.dropout_active = true;
        else w.range_sensor.dropout_active = false;
        hal_t h = hal_host_create(&w);
        range_sample_t r; imu_sample_t s;
        hal_status_t rs = h.range_read(h.ctx, &r);
        hal_status_t is = h.imu_read(h.ctx, &s);
        r.status = rs;
        s.status = is;
        estimator_output_t out = estimator_tick(&e, &r, &s);
        t_s[i] = w.clock_now_s;
        fused[i] = out.fused_speed_mps;
    }
    double true_v_final = w.cart.true_velocity_mps;
    double settle = find_settling_time(t_s, fused, N_TICKS, true_v_final, 0.1);
    printf("dropout-recovery settle = %.2f s\n", settle);
    CHECK(settle >= 0.0); /* it does reconverge within the run */
}

TEST(test_response_lag_after_step_change_in_true_velocity) {
    hal_host_world_t w;
    /* Real 1m/1m rig envelope. Bounded ramp-then-hold profile, not a
       sustained ramp: rest until the step at t=2.0s, a brief 0.2s pulse
       at 1.0 m/s^2 to bring true velocity up to a new, realistic 0.2 m/s
       plateau (well under v_cap=0.6), then zero acceleration for the
       remaining 2.8s so ground truth actually stabilizes. This gives
       find_settling_time real headroom to detect convergence, instead of
       chasing a target that is still linearly ramping at the last tick
       (the prior sustained-5.0-m/s^2-for-3s profile forced the settle
       check to pass only within ~1 tick of run-end regardless of filter
       quality). */
    hal_host_world_init(&w, 1.0, 1.0, 3);
    estimator_t e;
    estimator_init(&e, 0.90);

    int step_tick = 200;       /* t = 2.0s: when the pulse begins */
    int ramp_ticks = 20;       /* 0.2s pulse duration */
    double pulse_accel = 1.0;  /* m/s^2, applied only during the pulse */

    double t_s[N_TICKS]; double fused[N_TICKS];
    for (int i = 0; i < N_TICKS; i++) {
        double true_accel = 0.0;
        if (i >= step_tick && i < step_tick + ramp_ticks) true_accel = pulse_accel;
        hal_host_world_tick(&w, true_accel, 0.0, TICK_DT);
        hal_t h = hal_host_create(&w);
        range_sample_t r; imu_sample_t s;
        hal_status_t rs = h.range_read(h.ctx, &r);
        hal_status_t is = h.imu_read(h.ctx, &s);
        r.status = rs;
        s.status = is;
        estimator_output_t out = estimator_tick(&e, &r, &s);
        t_s[i] = w.clock_now_s;
        fused[i] = out.fused_speed_mps;
    }
    double true_v_final = w.cart.true_velocity_mps; /* the held plateau velocity */
    double settle = find_settling_time(t_s, fused, N_TICKS, true_v_final, 0.05);
    printf("step-response settle = %.2f s\n", settle);
    CHECK(settle >= 0.0);
    /* Real margin, not a last-tick technicality: settle with at least 1s
       of held-plateau headroom before the run ends. */
    CHECK(settle < t_s[N_TICKS - 1] - 1.0);
}

TEST(test_imu_bias_drift_stays_bounded_by_tof_corrections) {
    hal_host_world_t w;
    hal_host_world_init(&w, 1.0, 1.0, 9); /* real 1m/1m rig envelope, see note above */
    sim_imu_set_accel_bias(&w.imu, 0.3); /* persistent IMU accel bias */
    estimator_t e;
    estimator_init(&e, 0.90);

    metrics_accum_t fused_m;
    metrics_init(&fused_m);
    for (int i = 0; i < N_TICKS; i++) {
        hal_host_world_tick(&w, 0.03, 0.0, TICK_DT);
        hal_t h = hal_host_create(&w);
        range_sample_t r; imu_sample_t s;
        hal_status_t rs = h.range_read(h.ctx, &r);
        hal_status_t is = h.imu_read(h.ctx, &s);
        r.status = rs;
        s.status = is;
        estimator_output_t out = estimator_tick(&e, &r, &s);
        metrics_add(&fused_m, out.fused_speed_mps, w.cart.true_velocity_mps);
    }
    /* Bounded, not diverging without limit, thanks to periodic ToF
       correction — this is the point of fusion over pure IMU integration. */
    printf("bias-drift FUSED RMSE = %.4f m/s\n", metrics_rmse(&fused_m));
    CHECK(metrics_rmse(&fused_m) < 0.5);
}

int main(void) {
    RUN_TEST(test_fused_rmse_bias_variance_under_nominal_noise);
    RUN_TEST(test_dropout_recovery_fused_reconverges_after_tof_gap);
    RUN_TEST(test_response_lag_after_step_change_in_true_velocity);
    RUN_TEST(test_imu_bias_drift_stays_bounded_by_tof_corrections);
    TEST_SUMMARY();
    return 0;
}
