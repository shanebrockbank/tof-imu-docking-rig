#include "estimation/gravity_compensation.h"
#include <math.h>

double gravity_compensate(double a_x_mps2, double a_z_mps2, double theta_rad) {
    return a_x_mps2 * cos(theta_rad) - a_z_mps2 * sin(theta_rad);
}
