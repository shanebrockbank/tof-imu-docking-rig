#include "tests/test_framework.h"
#include "common/metrics.h"

TEST(test_rmse_bias_variance_on_known_errors) {
    metrics_accum_t m;
    metrics_init(&m);
    metrics_add(&m, 1.0, 0.0); /* error 1 */
    metrics_add(&m, 3.0, 0.0); /* error 3 */
    /* errors {1,3}: mean=2, mean_sq=(1+9)/2=5, rmse=sqrt(5), variance=5-4=1 */
    CHECK_NEAR(metrics_mean_bias(&m), 2.0, 1e-9);
    CHECK_NEAR(metrics_rmse(&m), 2.2360679775, 1e-6);
    CHECK_NEAR(metrics_variance(&m), 1.0, 1e-9);
}

TEST(test_rmse_squared_equals_bias_squared_plus_variance) {
    metrics_accum_t m;
    metrics_init(&m);
    double estimates[] = { 0.9, 1.2, 1.0, 0.8, 1.3 };
    double truth = 1.0;
    for (int i = 0; i < 5; i++) metrics_add(&m, estimates[i], truth);
    double rmse = metrics_rmse(&m);
    double bias = metrics_mean_bias(&m);
    double var = metrics_variance(&m);
    CHECK_NEAR(rmse * rmse, bias * bias + var, 1e-9);
}

TEST(test_jitter_rms_zero_for_constant_signal) {
    jitter_accum_t j;
    jitter_init(&j);
    for (int i = 0; i < 5; i++) jitter_add(&j, 0.5);
    CHECK_NEAR(jitter_rms(&j), 0.0, 1e-12);
}

TEST(test_jitter_rms_nonzero_for_alternating_signal) {
    jitter_accum_t j;
    jitter_init(&j);
    double vals[] = { 0.0, 1.0, 0.0, 1.0, 0.0 };
    for (int i = 0; i < 5; i++) jitter_add(&j, vals[i]);
    CHECK_NEAR(jitter_rms(&j), 1.0, 1e-9);
}

TEST(test_settling_time_finds_first_stable_crossing) {
    double t[]  = { 0.0, 0.1, 0.2, 0.3, 0.4, 0.5 };
    double v[]  = { 5.0, 3.0, 1.05, 0.98, 1.01, 1.0 };
    double settle = find_settling_time(t, v, 6, 1.0, 0.1);
    CHECK_NEAR(settle, 0.2, 1e-9);
}

TEST(test_settling_time_returns_negative_one_if_never_settles) {
    double t[] = { 0.0, 0.1, 0.2 };
    double v[] = { 5.0, 5.0, 5.0 };
    CHECK_NEAR(find_settling_time(t, v, 3, 1.0, 0.1), -1.0, 1e-12);
}

int main(void) {
    RUN_TEST(test_rmse_bias_variance_on_known_errors);
    RUN_TEST(test_rmse_squared_equals_bias_squared_plus_variance);
    RUN_TEST(test_jitter_rms_zero_for_constant_signal);
    RUN_TEST(test_jitter_rms_nonzero_for_alternating_signal);
    RUN_TEST(test_settling_time_finds_first_stable_crossing);
    RUN_TEST(test_settling_time_returns_negative_one_if_never_settles);
    TEST_SUMMARY();
    return 0;
}
