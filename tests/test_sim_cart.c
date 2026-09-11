#include "tests/test_framework.h"
#include "sim/sim_cart.h"

TEST(test_init_sets_initial_state) {
    sim_cart_t c;
    sim_cart_init(&c, 1.0, 1.0);
    CHECK_NEAR(c.true_range_m, 1.0, 1e-12);
    CHECK_NEAR(c.true_velocity_mps, 0.0, 1e-12);
}

TEST(test_positive_accel_increases_velocity_and_decreases_range) {
    sim_cart_t c;
    sim_cart_init(&c, 1.0, 1.0);
    sim_cart_step(&c, 0.5, 0.1); /* v += 0.05 -> 0.05; r -= 0.05*0.1 = 0.005 */
    CHECK_NEAR(c.true_velocity_mps, 0.05, 1e-9);
    CHECK_NEAR(c.true_range_m, 1.0 - 0.005, 1e-9);
}

TEST(test_range_clamped_at_zero_on_overshoot) {
    sim_cart_t c;
    sim_cart_init(&c, 0.01, 1.0);
    sim_cart_step(&c, 0.0, 0.01); /* velocity 0, no motion yet; force velocity via prior accel */
    sim_cart_step(&c, 100.0, 0.1); /* large accel -> large velocity -> range would go negative */
    CHECK(c.true_range_m >= 0.0);
}

TEST(test_range_clamped_at_track_length) {
    sim_cart_t c;
    sim_cart_init(&c, 0.0, 1.0);
    sim_cart_step(&c, -100.0, 0.1); /* negative accel -> negative velocity -> range grows past track */
    CHECK(c.true_range_m <= 1.0);
}

int main(void) {
    RUN_TEST(test_init_sets_initial_state);
    RUN_TEST(test_positive_accel_increases_velocity_and_decreases_range);
    RUN_TEST(test_range_clamped_at_zero_on_overshoot);
    RUN_TEST(test_range_clamped_at_track_length);
    TEST_SUMMARY();
    return 0;
}
