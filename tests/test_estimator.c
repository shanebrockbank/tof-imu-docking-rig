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
    range_sample_t r1 = make_range(0.05, 0.95, HAL_OK); /* -0.05m over 0.05s -> raw = -1.0 (moving away) is wrong sign; use approach */
    range_sample_t r1b = make_range(0.05, 0.9, HAL_OK); /* -0.1m over 0.05s -> raw = +2.0 m/s toward target */
    estimator_output_t out = estimator_tick(&e, &r1b, &imu1);
    CHECK(!out.raw_speed.is_stale);
    CHECK_NEAR(out.raw_speed.value_mps, 2.0, 1e-9);
    /* v_pred stayed ~0 (level, no accel), corrected toward 2.0 with alpha=0.5 */
    CHECK_NEAR(out.fused_speed_mps, 0.5 * 0.0 + 0.5 * 2.0, 1e-9);
    (void)r1;
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

int main(void) {
    RUN_TEST(test_first_tick_produces_stale_raw_and_zero_fused);
    RUN_TEST(test_no_new_tof_holds_raw_stale_but_imu_still_propagates);
    RUN_TEST(test_fresh_tof_sample_corrects_fused_toward_raw);
    RUN_TEST(test_imu_fault_holds_fused_speed);
    TEST_SUMMARY();
    return 0;
}
