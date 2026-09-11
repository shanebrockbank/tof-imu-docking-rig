#include "estimation/orientation_1d.h"
#include "common/dt_validation.h"

void orientation_1d_init(orientation_1d_t *o) {
    o->theta_rad = 0.0;
}

void orientation_1d_update(orientation_1d_t *o, double omega_y_rps, double dt) {
    if (!dt_is_valid(dt)) {
        return; /* hold last state */
    }
    o->theta_rad += omega_y_rps * dt;
}
