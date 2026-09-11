#include "tests/test_framework.h"
#include "sim/sim_imu.h"
#include <math.h>

TEST(test_level_noise_free_accel_x_equals_true_accel) {
    sim_imu_t s;
    sim_imu_init(&s, 0.01, 0.0, 0.0, 1);
    imu_sample_t out;
    timestamp_t t0 = { 0.0 };
    sim_imu_sample(&s, 0.3, 0.0 /* theta */, 0.0, t0, &out);
    CHECK_NEAR(out.accel_mps2[0], 0.3, 1e-9);
    CHECK_NEAR(out.accel_mps2[2], SIM_GRAVITY_MPS2, 1e-9);
}

TEST(test_pitched_accel_matches_derived_rotation_model) {
    sim_imu_t s;
    sim_imu_init(&s, 0.01, 0.0, 0.0, 1);
    imu_sample_t out;
    timestamp_t t0 = { 0.0 };
    double theta = 0.1;
    sim_imu_sample(&s, 0.0, theta, 0.0, t0, &out);
    CHECK_NEAR(out.accel_mps2[0], SIM_GRAVITY_MPS2 * sin(theta), 1e-9);
    CHECK_NEAR(out.accel_mps2[2], SIM_GRAVITY_MPS2 * cos(theta), 1e-9);
}

TEST(test_gyro_y_matches_true_omega) {
    sim_imu_t s;
    sim_imu_init(&s, 0.01, 0.0, 0.0, 1);
    imu_sample_t out;
    timestamp_t t0 = { 0.0 };
    sim_imu_sample(&s, 0.0, 0.0, 0.25, t0, &out);
    CHECK_NEAR(out.gyro_rps[1], 0.25, 1e-9);
}

TEST(test_no_sample_before_period_elapses) {
    sim_imu_t s;
    sim_imu_init(&s, 0.01, 0.0, 0.0, 1);
    imu_sample_t out;
    timestamp_t t0 = { 0.0 };
    sim_imu_sample(&s, 0.0, 0.0, 0.0, t0, &out);
    timestamp_t t1 = { 0.001 };
    CHECK(sim_imu_sample(&s, 0.0, 0.0, 0.0, t1, &out) == HAL_NO_NEW_DATA);
}

TEST(test_accel_bias_offsets_x_axis_only) {
    sim_imu_t s;
    sim_imu_init(&s, 0.01, 0.0, 0.0, 1);
    sim_imu_set_accel_bias(&s, 0.2);
    imu_sample_t out;
    timestamp_t t0 = { 0.0 };
    sim_imu_sample(&s, 0.0, 0.0, 0.0, t0, &out);
    CHECK_NEAR(out.accel_mps2[0], 0.2, 1e-9);
    CHECK_NEAR(out.accel_mps2[2], SIM_GRAVITY_MPS2, 1e-9);
}

int main(void) {
    RUN_TEST(test_level_noise_free_accel_x_equals_true_accel);
    RUN_TEST(test_pitched_accel_matches_derived_rotation_model);
    RUN_TEST(test_gyro_y_matches_true_omega);
    RUN_TEST(test_no_sample_before_period_elapses);
    RUN_TEST(test_accel_bias_offsets_x_axis_only);
    TEST_SUMMARY();
    return 0;
}
