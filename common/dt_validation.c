#include "common/dt_validation.h"

bool dt_is_valid(double dt) {
    return dt > 0.0 && dt <= DT_MAX_S;
}

double dt_between(timestamp_t earlier, timestamp_t later) {
    return later.t_s - earlier.t_s;
}
