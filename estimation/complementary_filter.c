#include "estimation/complementary_filter.h"
#include "common/dt_validation.h"

void complementary_filter_init(complementary_filter_t *f, double alpha) {
    f->v_est_mps = 0.0;
    f->alpha = alpha;
}

void complementary_filter_predict(complementary_filter_t *f, double a_longitudinal_mps2, double dt) {
    if (!dt_is_valid(dt)) {
        return; /* hold last state */
    }
    f->v_est_mps = f->v_est_mps + a_longitudinal_mps2 * dt;
}

void complementary_filter_correct(complementary_filter_t *f, double v_tof_mps) {
    f->v_est_mps = f->alpha * f->v_est_mps + (1.0 - f->alpha) * v_tof_mps;
}
