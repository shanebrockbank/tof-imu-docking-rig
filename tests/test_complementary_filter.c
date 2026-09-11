#include "tests/test_framework.h"
#include "estimation/complementary_filter.h"

TEST(test_init_starts_at_zero_velocity) {
    complementary_filter_t f;
    complementary_filter_init(&f, 0.9);
    CHECK_NEAR(f.v_est_mps, 0.0, 1e-12);
}

TEST(test_predict_integrates_accel_over_dt) {
    complementary_filter_t f;
    complementary_filter_init(&f, 0.9);
    complementary_filter_predict(&f, 0.5, 0.1);
    CHECK_NEAR(f.v_est_mps, 0.05, 1e-9);
}

TEST(test_predict_invalid_dt_holds_state) {
    complementary_filter_t f;
    complementary_filter_init(&f, 0.9);
    complementary_filter_predict(&f, 0.5, 0.1);
    double before = f.v_est_mps;
    complementary_filter_predict(&f, 0.5, -0.01);
    complementary_filter_predict(&f, 0.5, 3.0);
    CHECK_NEAR(f.v_est_mps, before, 1e-12);
}

TEST(test_correct_blends_prediction_toward_tof_speed) {
    complementary_filter_t f;
    complementary_filter_init(&f, 0.9);
    f.v_est_mps = 1.0; /* pretend prediction landed here */
    complementary_filter_correct(&f, 0.0); /* ToF says 0.0 */
    CHECK_NEAR(f.v_est_mps, 0.9 * 1.0 + 0.1 * 0.0, 1e-9);
}

TEST(test_alpha_one_ignores_tof_entirely) {
    complementary_filter_t f;
    complementary_filter_init(&f, 1.0);
    f.v_est_mps = 0.7;
    complementary_filter_correct(&f, 5.0);
    CHECK_NEAR(f.v_est_mps, 0.7, 1e-9);
}

int main(void) {
    RUN_TEST(test_init_starts_at_zero_velocity);
    RUN_TEST(test_predict_integrates_accel_over_dt);
    RUN_TEST(test_predict_invalid_dt_holds_state);
    RUN_TEST(test_correct_blends_prediction_toward_tof_speed);
    RUN_TEST(test_alpha_one_ignores_tof_entirely);
    TEST_SUMMARY();
    return 0;
}
