#include "guidance/pd_controller.h"
#include "common/dt_validation.h"

static double clamp11(double x) {
    if (x > 1.0) return 1.0;
    if (x < -1.0) return -1.0;
    return x;
}

void pd_controller_init(pd_controller_t *c, double kp, double kd, double tau_d) {
    c->kp = kp;
    c->kd = kd;
    c->tau_d = tau_d;
    c->prev_error = 0.0;
    c->has_prev_error = false;
    c->d_filtered = 0.0;
}

pd_output_t pd_controller_update(pd_controller_t *c, double speed_error, double dt) {
    double d_raw = 0.0;
    bool have_derivative = false;

    if (c->has_prev_error && dt_is_valid(dt)) {
        d_raw = (speed_error - c->prev_error) / dt;
        have_derivative = true;
        double ema_alpha = dt / (c->tau_d + dt);
        c->d_filtered = ema_alpha * d_raw + (1.0 - ema_alpha) * c->d_filtered;
    }
    /* invalid dt or no previous error yet: d_raw stays 0, d_filtered holds */

    c->prev_error = speed_error;
    c->has_prev_error = true;

    double p_term = c->kp * speed_error;
    pd_output_t out;
    out.control_output_unfiltered = clamp11(p_term + (have_derivative ? c->kd * d_raw : 0.0));
    out.control_output_filtered = clamp11(p_term + c->kd * c->d_filtered);
    return out;
}
