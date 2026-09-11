#ifndef COMMON_METRICS_H
#define COMMON_METRICS_H

#include <stdbool.h>

typedef struct {
    double sum_error;
    double sum_sq_error;
    int n;
} metrics_accum_t;

void metrics_init(metrics_accum_t *m);
/* error = estimate - truth. Pass truth=0 to accumulate raw-signal stats
   (mean_bias becomes the signal's mean, variance becomes its variance). */
void metrics_add(metrics_accum_t *m, double estimate, double truth);
double metrics_rmse(const metrics_accum_t *m);
double metrics_mean_bias(const metrics_accum_t *m);
double metrics_variance(const metrics_accum_t *m); /* RMSE^2 = bias^2 + variance */

typedef struct {
    double last_value;
    bool has_prev;
    double sum_sq_diff;
    int n;
} jitter_accum_t;

void jitter_init(jitter_accum_t *j);
void jitter_add(jitter_accum_t *j, double value);
double jitter_rms(const jitter_accum_t *j); /* RMS of consecutive-sample differences */

/* First t_s[i] after which values[] stays within tolerance of target for
   every remaining sample; -1.0 if it never does. */
double find_settling_time(const double *t_s, const double *values, int n, double target, double tolerance);

#endif /* COMMON_METRICS_H */
