#ifndef GUIDANCE_ACTUATOR_MAPPING_H
#define GUIDANCE_ACTUATOR_MAPPING_H

/* This servo acts as a visual glideslope-error gauge, not a physical
   actuator — the cart is hand-pushed and the servo has no authority over
   its motion (docs/design.md §7.3, §9). */
double actuator_map_to_servo_deg(double control_output);

#endif /* GUIDANCE_ACTUATOR_MAPPING_H */
