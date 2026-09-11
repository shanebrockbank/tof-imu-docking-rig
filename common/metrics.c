#include "common/metrics.h"
#include <math.h>
#include <stdbool.h>

void metrics_init(metrics_accum_t *m) {
    m->sum_error = 0.0;
    m->sum_sq_error = 0.0;
    m->n = 0;
}

void metrics_add(metrics_accum_t *m, double estimate, double truth) {
    double error = estimate - truth;
    m->sum_error += error;
    m->sum_sq_error += error * error;
    m->n += 1;
}

double metrics_rmse(const metrics_accum_t *m) {
    if (m->n == 0) return 0.0;
    return sqrt(m->sum_sq_error / m->n);
}

double metrics_mean_bias(const metrics_accum_t *m) {
    if (m->n == 0) return 0.0;
    return m->sum_error / m->n;
}

double metrics_variance(const metrics_accum_t *m) {
    if (m->n == 0) return 0.0;
    double bias = metrics_mean_bias(m);
    double mean_sq = m->sum_sq_error / m->n;
    return mean_sq - bias * bias;
}

void jitter_init(jitter_accum_t *j) {
    j->last_value = 0.0;
    j->has_prev = false;
    j->sum_sq_diff = 0.0;
    j->n = 0;
}

void jitter_add(jitter_accum_t *j, double value) {
    if (j->has_prev) {
        double diff = value - j->last_value;
        j->sum_sq_diff += diff * diff;
        j->n += 1;
    }
    j->last_value = value;
    j->has_prev = true;
}

double jitter_rms(const jitter_accum_t *j) {
    if (j->n == 0) return 0.0;
    return sqrt(j->sum_sq_diff / j->n);
}

double find_settling_time(const double *t_s, const double *values, int n, double target, double tolerance) {
    for (int i = 0; i < n; i++) {
        bool stays_within = true;
        for (int k = i; k < n; k++) {
            if (fabs(values[k] - target) > tolerance) {
                stays_within = false;
                break;
            }
        }
        if (stays_within) return t_s[i];
    }
    return -1.0;
}
