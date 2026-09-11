#include "tests/test_framework.h"
#include "guidance/v_safe.h"

TEST(test_v_safe_follows_sqrt_shape_in_midrange) {
    /* k*sqrt(0.36) = 0.5*0.6 = 0.3, below the 0.6 cap */
    CHECK_NEAR(v_safe(0.36), 0.3, 1e-9);
}

TEST(test_v_safe_capped_near_start_of_approach) {
    CHECK_NEAR(v_safe(1.0), 0.5, 1e-9); /* 0.5*sqrt(1)=0.5, under cap */
    CHECK_NEAR(v_safe(10.0), 0.6, 1e-9); /* 0.5*sqrt(10)=1.58 -> capped at 0.6 */
}

TEST(test_v_safe_floored_near_contact) {
    CHECK_NEAR(v_safe(0.0), 0.03, 1e-9);
    CHECK_NEAR(v_safe(0.0001), 0.03, 1e-9); /* 0.5*sqrt(0.0001)=0.005 -> floored */
}

TEST(test_v_safe_negative_range_treated_as_zero) {
    CHECK_NEAR(v_safe(-0.5), 0.03, 1e-9);
}

int main(void) {
    RUN_TEST(test_v_safe_follows_sqrt_shape_in_midrange);
    RUN_TEST(test_v_safe_capped_near_start_of_approach);
    RUN_TEST(test_v_safe_floored_near_contact);
    RUN_TEST(test_v_safe_negative_range_treated_as_zero);
    TEST_SUMMARY();
    return 0;
}
