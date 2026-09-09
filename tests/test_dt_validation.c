#include "tests/test_framework.h"
#include "common/dt_validation.h"

TEST(test_positive_dt_within_bound_is_valid) {
    CHECK(dt_is_valid(0.01));
    CHECK(dt_is_valid(1.99));
}

TEST(test_zero_or_negative_dt_is_invalid) {
    CHECK(!dt_is_valid(0.0));
    CHECK(!dt_is_valid(-0.01));
}

TEST(test_dt_over_max_is_invalid) {
    CHECK(!dt_is_valid(DT_MAX_S + 0.001));
    CHECK(dt_is_valid(DT_MAX_S));
}

TEST(test_dt_between_computes_forward_difference) {
    timestamp_t a = { 1.0 };
    timestamp_t b = { 1.05 };
    CHECK_NEAR(dt_between(a, b), 0.05, 1e-9);
}

int main(void) {
    RUN_TEST(test_positive_dt_within_bound_is_valid);
    RUN_TEST(test_zero_or_negative_dt_is_invalid);
    RUN_TEST(test_dt_over_max_is_invalid);
    RUN_TEST(test_dt_between_computes_forward_difference);
    TEST_SUMMARY();
    return 0;
}
