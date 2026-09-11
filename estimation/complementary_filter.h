#ifndef ESTIMATION_COMPLEMENTARY_FILTER_H
#define ESTIMATION_COMPLEMENTARY_FILTER_H

/* V1 complementary filter (docs/design.md §6.5). A Kalman filter is an
   explicit, named V2 possibility (DEBT-2) — not implemented here.
   The IMU provides only short-term propagation between valid ToF
   corrections; this filter does not rely on IMU integration for long-term
   accuracy — every correct() call pulls the estimate back toward the
   ToF-derived ground truth. */
typedef struct {
    double v_est_mps;
    double alpha; /* weight on IMU-propagated prediction at each correction */
} complementary_filter_t;

void complementary_filter_init(complementary_filter_t *f, double alpha);

/* v_pred = v_est + a_longitudinal*dt. On invalid dt (docs/design.md §6.4),
   holds v_est_mps unchanged (zero propagation) instead of integrating
   over a bad dt. Call once per IMU tick. */
void complementary_filter_predict(complementary_filter_t *f, double a_longitudinal_mps2, double dt);

/* v_est = alpha*v_est + (1-alpha)*v_tof. Call only on ticks with a fresh
   (non-stale) ToF-derived speed, AFTER predict() has run for this tick. */
void complementary_filter_correct(complementary_filter_t *f, double v_tof_mps);

#endif /* ESTIMATION_COMPLEMENTARY_FILTER_H */
