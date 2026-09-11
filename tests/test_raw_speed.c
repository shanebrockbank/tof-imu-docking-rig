#include "tests/test_framework.h"
#include "estimation/raw_speed.h"

TEST(test_first_sample_is_stale_no_prior_to_difference) {
    raw_speed_estimator_t e;
    raw_speed_init(&e);
    timestamp_t t0 = { 0.0 };
    raw_speed_on_new_range(&e, 1.0, t0);
    CHECK(e.speed.is_stale);
}

TEST(test_second_sample_computes_backward_difference) {
    raw_speed_estimator_t e;
    raw_speed_init(&e);
    timestamp_t t0 = { 0.0 };
    timestamp_t t1 = { 0.05 };
    raw_speed_on_new_range(&e, 1.0, t0);
    raw_speed_on_new_range(&e, 0.95, t1); /* range decreased 0.05 over 0.05s -> +1.0 m/s */
    CHECK(!e.speed.is_stale);
    CHECK_NEAR(e.speed.value_mps, 1.0, 1e-9);
}

TEST(test_mark_stale_holds_last_value) {
    raw_speed_estimator_t e;
    raw_speed_init(&e);
    timestamp_t t0 = { 0.0 };
    timestamp_t t1 = { 0.05 };
    raw_speed_on_new_range(&e, 1.0, t0);
    raw_speed_on_new_range(&e, 0.95, t1);
    double held = e.speed.value_mps;
    raw_speed_mark_stale(&e);
    CHECK(e.speed.is_stale);
    CHECK_NEAR(e.speed.value_mps, held, 1e-12);
}

TEST(test_invalid_dt_holds_and_flags_stale_instead_of_dividing) {
    raw_speed_estimator_t e;
    raw_speed_init(&e);
    timestamp_t t0 = { 0.0 };
    raw_speed_on_new_range(&e, 1.0, t0);
    raw_speed_on_new_range(&e, 0.9, t0); /* duplicate timestamp: dt=0 */
    CHECK(e.speed.is_stale);
}

int main(void) {
    RUN_TEST(test_first_sample_is_stale_no_prior_to_difference);
    RUN_TEST(test_second_sample_computes_backward_difference);
    RUN_TEST(test_mark_stale_holds_last_value);
    RUN_TEST(test_invalid_dt_holds_and_flags_stale_instead_of_dividing);
    TEST_SUMMARY();
    return 0;
}
