#include "tests/test_framework.h"
#include "sim/sim_noise.h"
#include <math.h>

TEST(test_same_seed_reproduces_same_sequence) {
    sim_noise_t a, b;
    sim_noise_seed(&a, 42);
    sim_noise_seed(&b, 42);
    for (int i = 0; i < 20; i++) {
        CHECK_NEAR(sim_noise_gaussian(&a, 0, 1), sim_noise_gaussian(&b, 0, 1), 1e-12);
    }
}

TEST(test_zero_stddev_returns_mean_exactly) {
    sim_noise_t n;
    sim_noise_seed(&n, 7);
    for (int i = 0; i < 10; i++) {
        CHECK_NEAR(sim_noise_gaussian(&n, 3.5, 0.0), 3.5, 1e-12);
    }
}

TEST(test_sample_mean_converges_toward_requested_mean) {
    sim_noise_t n;
    sim_noise_seed(&n, 123);
    double sum = 0.0;
    int count = 20000;
    for (int i = 0; i < count; i++) sum += sim_noise_gaussian(&n, 2.0, 0.5);
    CHECK_NEAR(sum / count, 2.0, 0.05);
}

int main(void) {
    RUN_TEST(test_same_seed_reproduces_same_sequence);
    RUN_TEST(test_zero_stddev_returns_mean_exactly);
    RUN_TEST(test_sample_mean_converges_toward_requested_mean);
    TEST_SUMMARY();
    return 0;
}
