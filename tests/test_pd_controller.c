#include "tests/test_framework.h"
#include "guidance/pd_controller.h"
#include <math.h>

TEST(test_first_call_has_no_derivative_contribution) {
    pd_controller_t c;
    pd_controller_init(&c, 1.0, 1.0, 0.05);
    pd_output_t out = pd_controller_update(&c, 0.2, 0.01);
    CHECK_NEAR(out.control_output_unfiltered, 0.2, 1e-9); /* pure P term */
}

TEST(test_proportional_term_scales_with_kp) {
    pd_controller_t c;
    pd_controller_init(&c, 2.0, 0.0, 0.05);
    pd_output_t out = pd_controller_update(&c, 0.1, 0.01);
    CHECK_NEAR(out.control_output_filtered, 0.2, 1e-9);
}

TEST(test_output_saturates_to_plus_minus_one) {
    pd_controller_t c;
    pd_controller_init(&c, 10.0, 0.0, 0.05);
    pd_output_t out = pd_controller_update(&c, 5.0, 0.01);
    CHECK_NEAR(out.control_output_filtered, 1.0, 1e-9);
    pd_output_t out2 = pd_controller_update(&c, -5.0, 0.01);
    CHECK_NEAR(out2.control_output_filtered, -1.0, 1e-9);
}

TEST(test_invalid_dt_falls_back_to_proportional_only_derivative) {
    pd_controller_t c;
    pd_controller_init(&c, 1.0, 5.0, 0.05);
    pd_controller_update(&c, 0.0, 0.01);
    pd_output_t out = pd_controller_update(&c, 1.0, 0.0); /* dt=0, invalid */
    CHECK_NEAR(out.control_output_unfiltered, 1.0, 1e-9); /* kp*1.0, no D term */
}

TEST(test_unfiltered_d_term_amplifies_noise_more_than_filtered) {
    pd_controller_t c;
    /* D-only, isolate D-term behavior. NOTE: kd=0.1 here (not 1.0) — with
       kd=1.0 the huge raw derivative swings (~100 units, from dt=0.01 and
       alternating +/-0.5 error) push BOTH the unfiltered and the
       low-pass-filtered D-term output past the +/-1 saturation clamp on
       every step, making the two sums equal and the assertion below false.
       kd=0.1 keeps kd*d_filtered under the clamp while kd*d_raw still
       saturates, which is what actually demonstrates the noise-amplification
       difference the low-pass filter is meant to fix. */
    pd_controller_init(&c, 0.0, 0.1, 0.05);
    double errors[] = { 0.0, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5 }; /* noisy alternating error */
    double sum_sq_unfiltered = 0.0, sum_sq_filtered = 0.0;
    for (int i = 0; i < 8; i++) {
        pd_output_t out = pd_controller_update(&c, errors[i], 0.01);
        sum_sq_unfiltered += out.control_output_unfiltered * out.control_output_unfiltered;
        sum_sq_filtered += out.control_output_filtered * out.control_output_filtered;
    }
    CHECK(sum_sq_unfiltered > sum_sq_filtered); /* the demonstrable D-term amplification + fix */
}

int main(void) {
    RUN_TEST(test_first_call_has_no_derivative_contribution);
    RUN_TEST(test_proportional_term_scales_with_kp);
    RUN_TEST(test_output_saturates_to_plus_minus_one);
    RUN_TEST(test_invalid_dt_falls_back_to_proportional_only_derivative);
    RUN_TEST(test_unfiltered_d_term_amplifies_noise_more_than_filtered);
    TEST_SUMMARY();
    return 0;
}
