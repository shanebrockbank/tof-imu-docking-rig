#include "tests/test_framework.h"
#include "sim/sim_range_sensor.h"

TEST(test_no_sample_before_first_period_elapses) {
    sim_range_sensor_t s;
    sim_range_sensor_init(&s, 0.05, 0.0, 1);
    range_sample_t out;
    timestamp_t t0 = { 0.0 };
    CHECK(sim_range_sensor_sample(&s, 1.0, t0, &out) == HAL_OK); /* first call always due */
    timestamp_t t1 = { 0.01 };
    CHECK(sim_range_sensor_sample(&s, 1.0, t1, &out) == HAL_NO_NEW_DATA);
}

TEST(test_sample_emitted_once_period_elapses) {
    sim_range_sensor_t s;
    sim_range_sensor_init(&s, 0.05, 0.0, 1);
    range_sample_t out;
    timestamp_t t0 = { 0.0 };
    sim_range_sensor_sample(&s, 1.0, t0, &out);
    timestamp_t t1 = { 0.05 };
    CHECK(sim_range_sensor_sample(&s, 0.9, t1, &out) == HAL_OK);
    CHECK_NEAR(out.range_m, 0.9, 1e-9);
    CHECK_NEAR(out.ts.t_s, 0.05, 1e-9);
}

TEST(test_dropout_forces_no_new_data_even_when_due) {
    sim_range_sensor_t s;
    sim_range_sensor_init(&s, 0.05, 0.0, 1);
    range_sample_t out;
    timestamp_t t0 = { 0.0 };
    sim_range_sensor_sample(&s, 1.0, t0, &out);
    sim_range_sensor_set_dropout(&s, true);
    timestamp_t t1 = { 0.5 };
    CHECK(sim_range_sensor_sample(&s, 0.5, t1, &out) == HAL_NO_NEW_DATA);
    sim_range_sensor_set_dropout(&s, false);
    timestamp_t t2 = { 0.55 };
    CHECK(sim_range_sensor_sample(&s, 0.5, t2, &out) == HAL_OK);
}

TEST(test_noise_free_sample_matches_true_range_exactly) {
    sim_range_sensor_t s;
    sim_range_sensor_init(&s, 0.05, 0.0, 1);
    range_sample_t out;
    timestamp_t t0 = { 0.0 };
    sim_range_sensor_sample(&s, 0.75, t0, &out);
    CHECK_NEAR(out.range_m, 0.75, 1e-12);
}

int main(void) {
    RUN_TEST(test_no_sample_before_first_period_elapses);
    RUN_TEST(test_sample_emitted_once_period_elapses);
    RUN_TEST(test_dropout_forces_no_new_data_even_when_due);
    RUN_TEST(test_noise_free_sample_matches_true_range_exactly);
    TEST_SUMMARY();
    return 0;
}
