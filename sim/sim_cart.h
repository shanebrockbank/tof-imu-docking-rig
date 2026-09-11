#ifndef SIM_SIM_CART_H
#define SIM_SIM_CART_H

typedef struct {
    double true_range_m;     /* distance to target, >= 0, decreases approaching */
    double true_velocity_mps; /* positive toward target */
    double track_length_m;
} sim_cart_t;

void sim_cart_init(sim_cart_t *c, double initial_range_m, double track_length_m);

/* true_accel_mps2: positive toward target. Integrates velocity then range.
   Clamps range to [0, track_length_m]. */
void sim_cart_step(sim_cart_t *c, double true_accel_mps2, double dt);

#endif /* SIM_SIM_CART_H */
