#include "tests/test_framework.h"
#include "estimation/orientation_1d.h"

TEST(test_init_starts_at_zero) {
    orientation_1d_t o;
    orientation_1d_init(&o);
    CHECK_NEAR(o.theta_rad, 0.0, 1e-12);
}

TEST(test_update_integrates_omega_over_dt) {
    orientation_1d_t o;
    orientation_1d_init(&o);
    orientation_1d_update(&o, 0.5, 0.1);
    CHECK_NEAR(o.theta_rad, 0.05, 1e-9);
    orientation_1d_update(&o, 0.5, 0.1);
    CHECK_NEAR(o.theta_rad, 0.10, 1e-9);
}

TEST(test_invalid_dt_holds_theta) {
    orientation_1d_t o;
    orientation_1d_init(&o);
    orientation_1d_update(&o, 0.5, 0.1);
    double before = o.theta_rad;
    orientation_1d_update(&o, 0.5, 0.0);   /* dt<=0 */
    orientation_1d_update(&o, 0.5, 3.0);   /* dt>DT_MAX_S */
    CHECK_NEAR(o.theta_rad, before, 1e-12);
}

int main(void) {
    RUN_TEST(test_init_starts_at_zero);
    RUN_TEST(test_update_integrates_omega_over_dt);
    RUN_TEST(test_invalid_dt_holds_theta);
    TEST_SUMMARY();
    return 0;
}
