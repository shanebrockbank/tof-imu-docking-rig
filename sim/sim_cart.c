#include "sim/sim_cart.h"

void sim_cart_init(sim_cart_t *c, double initial_range_m, double track_length_m) {
    c->true_range_m = initial_range_m;
    c->true_velocity_mps = 0.0;
    c->track_length_m = track_length_m;
}

void sim_cart_step(sim_cart_t *c, double true_accel_mps2, double dt) {
    c->true_velocity_mps += true_accel_mps2 * dt;
    c->true_range_m -= c->true_velocity_mps * dt; /* +velocity toward target shrinks range */
    if (c->true_range_m < 0.0) c->true_range_m = 0.0;
    if (c->true_range_m > c->track_length_m) c->true_range_m = c->track_length_m;
}
