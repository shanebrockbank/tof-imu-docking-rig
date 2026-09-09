#ifndef COMMON_DT_VALIDATION_H
#define COMMON_DT_VALIDATION_H

#include <stdbool.h>
#include "common/types.h"

/* Generous outer sanity bound for any computed dt (not a dropout-length
   limiter — see docs/design.md §6.4). */
#define DT_MAX_S 2.0

bool dt_is_valid(double dt);
double dt_between(timestamp_t earlier, timestamp_t later);

#endif /* COMMON_DT_VALIDATION_H */
