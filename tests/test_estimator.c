#include "tests/test_framework.h"
#include "estimation/estimator.h"

static imu_sample_t make_imu(double t, double a_x, double a_z, double gyro_y, hal_status_t status) {
    imu_sample_t s;
    s.accel_mps2[0] = a_x; s.accel_mps2[1] = 0; s.accel_mps2[2] = a_z;
    s.gyro_rps[0] = 0; s.gyro_rps[1] = gyro_y; s.gyro_rps[2] = 0;
    s.ts.t_s = t;
    s.status = status;
    return s;
}

static range_sample_t make_range(double t, double r, hal_status_t status) {
    range_sample_t s;
    s.range_m = r;
    s.ts.t_s = t;
    s.status = status;
    return s;
}

TEST(test_first_tick_produces_stale_raw_and_zero_fused) {
    estimator_t e;
    estimator_init(&e, 0.9);
    imu_sample_t imu = make_imu(0.0, 0.0, 9.81, 0.0, HAL_OK);
    range_sample_t r = make_range(0.0, 1.0, HAL_OK);
    estimator_output_t out = estimator_tick(&e, &r, &imu);
    CHECK(out.raw_speed.is_stale);
    CHECK_NEAR(out.fused_speed_mps, 0.0, 1e-9);
}

TEST(test_no_new_tof_holds_raw_stale_but_imu_still_propagates) {
    estimator_t e;
    estimator_init(&e, 0.9);
    imu_sample_t imu0 = make_imu(0.0, 0.0, 9.81, 0.0, HAL_OK);
    range_sample_t r0 = make_range(0.0, 1.0, HAL_OK);
    estimator_tick(&e, &r0, &imu0);

    imu_sample_t imu1 = make_imu(0.01, 0.5, 9.81, 0.0, HAL_OK); /* level, 0.5 m/s^2 forward */
    range_sample_t r1 = make_range(0.01, 1.0, HAL_NO_NEW_DATA);
    estimator_output_t out = estimator_tick(&e, &r1, &imu1);
    CHECK(out.raw_speed.is_stale);
    CHECK(out.fused_speed_mps > 0.0); /* IMU propagated velocity forward */
}

TEST(test_fresh_tof_sample_corrects_fused_toward_raw) {
    estimator_t e;
    estimator_init(&e, 0.5); /* heavy weight on ToF correction to make the test's expected pull visible */
    imu_sample_t imu0 = make_imu(0.0, 0.0, 9.81, 0.0, HAL_OK);
    range_sample_t r0 = make_range(0.0, 1.0, HAL_OK);
    estimator_tick(&e, &r0, &imu0);

    imu_sample_t imu1 = make_imu(0.05, 0.0, 9.81, 0.0, HAL_OK);
    range_sample_t r1b = make_range(0.05, 0.9, HAL_OK); /* -0.1m over 0.05s -> raw = +2.0 m/s toward target */
    estimator_output_t out = estimator_tick(&e, &r1b, &imu1);
    CHECK(!out.raw_speed.is_stale);
    CHECK_NEAR(out.raw_speed.value_mps, 2.0, 1e-9);
    /* v_pred stayed ~0 (level, no accel), corrected toward 2.0 with alpha=0.5 */
    CHECK_NEAR(out.fused_speed_mps, 0.5 * 0.0 + 0.5 * 2.0, 1e-9);
}

TEST(test_imu_fault_holds_fused_speed) {
    estimator_t e;
    estimator_init(&e, 0.9);
    imu_sample_t imu0 = make_imu(0.0, 0.0, 9.81, 0.0, HAL_OK);
    range_sample_t r0 = make_range(0.0, 1.0, HAL_OK);
    estimator_tick(&e, &r0, &imu0);

    imu_sample_t imu1 = make_imu(0.01, 0.5, 9.81, 0.0, HAL_FAULT);
    range_sample_t r1 = make_range(0.01, 1.0, HAL_NO_NEW_DATA);
    estimator_output_t out = estimator_tick(&e, &r1, &imu1);
    CHECK_NEAR(out.fused_speed_mps, 0.0, 1e-9); /* no propagation happened */
}

TEST(test_composed_predict_and_correct) {
    /* This test verifies that predict() and correct() compose properly
       in the same estimator_tick execution. It ensures that:
       1. predict() produces a nonzero v_pred from IMU acceleration
       2. correct() blends v_pred with v_tof correctly
       3. The order of execution (predict then correct) is preserved */

    estimator_t e;
    const double alpha = 0.6;
    estimator_init(&e, alpha);

    /* Tick 1: Establish initial range, no predict yet (first IMU sample) */
    imu_sample_t imu0 = make_imu(0.0, 0.0, 9.81, 0.0, HAL_OK);
    range_sample_t range0 = make_range(0.0, 10.0, HAL_OK);
    estimator_tick(&e, &range0, &imu0);

    /* Tick 2: IMU predict with nonzero acceleration, no new range (HAL_NO_NEW_DATA)
       - a_x = 1.0 m/s^2, a_z = 9.81, theta = 0
       - gravity_compensate(1.0, 9.81, 0) = 1.0*cos(0) - 9.81*sin(0) = 1.0
       - dt = 0.1 - 0.0 = 0.1 s
       - v_pred = 0.0 + 1.0*0.1 = 0.1 m/s */
    imu_sample_t imu1 = make_imu(0.1, 1.0, 9.81, 0.0, HAL_OK);
    range_sample_t range1 = make_range(0.1, 10.0, HAL_NO_NEW_DATA);
    estimator_output_t out1 = estimator_tick(&e, &range1, &imu1);
    double v_pred = out1.fused_speed_mps;
    CHECK_NEAR(v_pred, 0.1, 1e-9);

    /* Tick 3: Zero IMU acceleration (predict is no-op), fresh range sample
       - a_x = 0.0, so gravity_compensate returns 0
       - dt = 0.2 - 0.1 = 0.1 s
       - predict: v_est = 0.1 + 0*0.1 = 0.1 (unchanged)
       - range = 9.9 m (moved 0.1m closer in 0.2s total from tick 1)
       - v_tof = (last_range - current_range) / dt_range = (10.0 - 9.9) / (0.2 - 0.0) = 0.1 / 0.2 = 0.5 m/s
       - correct: v_est = alpha*v_pred + (1-alpha)*v_tof = 0.6*0.1 + 0.4*0.5 = 0.26 m/s */
    imu_sample_t imu2 = make_imu(0.2, 0.0, 9.81, 0.0, HAL_OK);
    range_sample_t range2 = make_range(0.2, 9.9, HAL_OK);
    estimator_output_t out2 = estimator_tick(&e, &range2, &imu2);

    double v_tof = out2.raw_speed.value_mps;
    CHECK_NEAR(v_tof, 0.5, 1e-9);

    double expected_fused = alpha * v_pred + (1.0 - alpha) * v_tof;
    CHECK_NEAR(out2.fused_speed_mps, expected_fused, 1e-9);
}

int main(void) {
    RUN_TEST(test_first_tick_produces_stale_raw_and_zero_fused);
    RUN_TEST(test_no_new_tof_holds_raw_stale_but_imu_still_propagates);
    RUN_TEST(test_fresh_tof_sample_corrects_fused_toward_raw);
    RUN_TEST(test_imu_fault_holds_fused_speed);
    RUN_TEST(test_composed_predict_and_correct);
    TEST_SUMMARY();
    return 0;
}
