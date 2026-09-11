#ifndef ESTIMATION_RAW_SPEED_H
#define ESTIMATION_RAW_SPEED_H

#include <stdbool.h>
#include "common/types.h"

typedef struct {
    double value_mps;
    bool is_stale; /* true = held from a previous tick, not recomputed this tick */
} raw_speed_t;

typedef struct {
    double last_range_m;
    timestamp_t last_ts;
    bool has_prev;
    raw_speed_t speed;
} raw_speed_estimator_t;

void raw_speed_init(raw_speed_estimator_t *e);

/* Call ONLY on ticks where a genuinely new ToF sample arrived
   (docs/design.md §6.1). Computes (r_k - r_{k-1})/(t_k - t_{k-1}) subject
   to dt validation; on an invalid dt or no previous sample yet, holds/
   flags stale instead of dividing by a bad or nonexistent dt. */
void raw_speed_on_new_range(raw_speed_estimator_t *e, double range_m, timestamp_t ts);

/* Call on every tick with no new ToF sample: holds value_mps, sets is_stale. */
void raw_speed_mark_stale(raw_speed_estimator_t *e);

#endif /* ESTIMATION_RAW_SPEED_H */
