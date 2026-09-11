#ifndef GUIDANCE_PD_CONTROLLER_H
#define GUIDANCE_PD_CONTROLLER_H

#include <stdbool.h>

/* Never knows about servo angles/degrees — output is normalized to
   [-1,+1] only (docs/design.md §7.2). Servo mapping lives in
   actuator_mapping.c. */
typedef struct {
    double kp;
    double kd;
    double tau_d; /* D-term low-pass time constant, seconds */
    double prev_error;
    bool has_prev_error;
    double d_filtered;
} pd_controller_t;

typedef struct {
    double control_output_unfiltered; /* saturated P + raw D, for comparison only */
    double control_output_filtered;   /* saturated P + low-pass-filtered D — the real output */
} pd_output_t;

void pd_controller_init(pd_controller_t *c, double kp, double kd, double tau_d);

/* On invalid dt (docs/design.md §6.4), the derivative term contributes 0
   for that tick (falls back to proportional-only) rather than dividing by
   a bad dt. */
pd_output_t pd_controller_update(pd_controller_t *c, double speed_error, double dt);

#endif /* GUIDANCE_PD_CONTROLLER_H */
