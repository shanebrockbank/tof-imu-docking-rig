#include "guidance/v_safe.h"
#include <math.h>

#define V_SAFE_K 0.5
#define V_SAFE_CAP_MPS 0.6
#define V_SAFE_FLOOR_MPS 0.03

double v_safe(double range_m) {
    if (range_m < 0.0) range_m = 0.0;
    double v = V_SAFE_K * sqrt(range_m);
    if (v > V_SAFE_CAP_MPS) v = V_SAFE_CAP_MPS;
    if (v < V_SAFE_FLOOR_MPS) v = V_SAFE_FLOOR_MPS;
    return v;
}
