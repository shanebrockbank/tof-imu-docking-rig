#include "guidance/actuator_mapping.h"

#define SERVO_CENTER_DEG 90.0
#define SERVO_HALF_RANGE_DEG 45.0

double actuator_map_to_servo_deg(double control_output) {
    if (control_output > 1.0) control_output = 1.0;
    if (control_output < -1.0) control_output = -1.0;
    return SERVO_CENTER_DEG + control_output * SERVO_HALF_RANGE_DEG;
}
