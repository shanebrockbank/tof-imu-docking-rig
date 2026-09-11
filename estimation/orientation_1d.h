#ifndef ESTIMATION_ORIENTATION_1D_H
#define ESTIMATION_ORIENTATION_1D_H

/* Cart is rail-constrained: no roll/yaw DOF exists to estimate, so
   orientation is a single scalar pitch about Y, not a 3D attitude
   (docs/design.md §3.1). theta feeds gravity_compensation.c only. */
typedef struct {
    double theta_rad;
} orientation_1d_t;

void orientation_1d_init(orientation_1d_t *o);

/* theta_k = theta_{k-1} + omega_y*dt. On invalid dt (docs/design.md §6.4),
   holds theta_rad unchanged rather than integrating over a bad dt. */
void orientation_1d_update(orientation_1d_t *o, double omega_y_rps, double dt);

#endif /* ESTIMATION_ORIENTATION_1D_H */
