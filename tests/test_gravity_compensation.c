#include "tests/test_framework.h"
#include "estimation/gravity_compensation.h"
#include "sim/sim_imu.h"
#include <math.h>

TEST(test_level_case_passes_ax_through_unchanged) {
    CHECK_NEAR(gravity_compensate(0.4, SIM_GRAVITY_MPS2, 0.0), 0.4, 1e-9);
}

TEST(test_recovers_true_accel_through_sim_imu_forward_model_at_various_pitches) {
    double thetas[] = { -0.3, -0.1, 0.0, 0.1, 0.3 };
    double true_accels[] = { -0.5, 0.0, 0.2, 0.6 };
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 4; j++) {
            double theta = thetas[i];
            double true_a = true_accels[j];
            double a_x =  true_a * cos(theta) + SIM_GRAVITY_MPS2 * sin(theta);
            double a_z = -true_a * sin(theta) + SIM_GRAVITY_MPS2 * cos(theta);
            CHECK_NEAR(gravity_compensate(a_x, a_z, theta), true_a, 1e-9);
        }
    }
}

int main(void) {
    RUN_TEST(test_level_case_passes_ax_through_unchanged);
    RUN_TEST(test_recovers_true_accel_through_sim_imu_forward_model_at_various_pitches);
    TEST_SUMMARY();
    return 0;
}
