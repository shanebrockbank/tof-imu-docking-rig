#ifndef ESTIMATION_GRAVITY_COMPENSATION_H
#define ESTIMATION_GRAVITY_COMPENSATION_H

/* a_longitudinal = a_x*cos(theta) - a_z*sin(theta), using the 1D pitch
   theta (docs/design.md §3.1, §6.2). Own named module per working
   agreement — not inlined into the fusion code. */
double gravity_compensate(double a_x_mps2, double a_z_mps2, double theta_rad);

#endif /* ESTIMATION_GRAVITY_COMPENSATION_H */
