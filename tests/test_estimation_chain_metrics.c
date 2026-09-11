#include "tests/test_framework.h"
#include "hal/hal_host.h"
#include "estimation/estimator.h"
#include "common/metrics.h"
#include <stdio.h>

#define TICK_DT 0.01
#define N_TICKS 500 /* 5 seconds */

TEST(test_fused_rmse_bias_variance_under_nominal_noise) {
    hal_host_world_t w;
    /* 50m/50m, not 1m/1m: sim_cart_step() clamps true_range_m to
       [0, track_length_m] but never clamps true_velocity_mps, so a 1m
       track is fully traversed well before N_TICKS=500 (5s) at this
       accel, after which ground-truth velocity keeps climbing past the
       point where range (and therefore RAW/FUSED) can reflect it. That
       wall-collision artifact — not filter quality — was what blew up
       RMSE/bias on first run; giving the cart room fixes the scenario
       without touching estimation code. */
    hal_host_world_init(&w, 50.0, 50.0, 42);
    estimator_t e;
    estimator_init(&e, 0.90);

    metrics_accum_t raw_m, fused_m;
    metrics_init(&raw_m);
    metrics_init(&fused_m);

    double true_accel = 0.2; /* constant gentle push toward target */
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
    CHECK(metrics_rmse(&fused_m) < metrics_rmse(&raw_m));
    CHECK(metrics_rmse(&fused_m) < 0.15);
    CHECK(fabs(metrics_mean_bias(&fused_m)) < 0.1);
}

TEST(test_dropout_recovery_fused_reconverges_after_tof_gap) {
    hal_host_world_t w;
    hal_host_world_init(&w, 50.0, 50.0, 7); /* see track-length note above */
    estimator_t e;
    estimator_init(&e, 0.90);

    double true_accel = 0.15;
    double t_s[N_TICKS]; double fused[N_TICKS];
    for (int i = 0; i < N_TICKS; i++) {
        bool in_gap = (i >= 100 && i < 130); /* ~300ms ToF gap, well under DT_MAX_S=2.0 */
        if (in_gap) {
            /* force dropout on the world's range sensor directly */
        }
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
    CHECK(settle >= 0.0); /* it does reconverge within the run */
}

TEST(test_response_lag_after_step_change_in_true_velocity) {
    hal_host_world_t w;
    /* 50m/50m: this scenario ramps to 15 m/s, covering ~22.5m in 3s —
       see track-length note in test 1 above. */
    hal_host_world_init(&w, 50.0, 50.0, 3);
    estimator_t e;
    estimator_init(&e, 0.90);

    double t_s[N_TICKS]; double fused[N_TICKS];
    for (int i = 0; i < N_TICKS; i++) {
        double true_accel = (i < 200) ? 0.0 : 5.0; /* step in acceleration -> ramp to a new velocity */
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
    double true_v_final = w.cart.true_velocity_mps;
    double settle = find_settling_time(t_s, fused, N_TICKS, true_v_final, 0.15);
    CHECK(settle >= 0.0);
    CHECK(settle < t_s[N_TICKS - 1]); /* settles before the run ends */
}

TEST(test_imu_bias_drift_stays_bounded_by_tof_corrections) {
    hal_host_world_t w;
    hal_host_world_init(&w, 50.0, 50.0, 9); /* see track-length note above */
    sim_imu_set_accel_bias(&w.imu, 0.3); /* persistent IMU accel bias */
    estimator_t e;
    estimator_init(&e, 0.90);

    metrics_accum_t fused_m;
    metrics_init(&fused_m);
    for (int i = 0; i < N_TICKS; i++) {
        hal_host_world_tick(&w, 0.1, 0.0, TICK_DT);
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
