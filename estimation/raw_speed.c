#include "estimation/raw_speed.h"
#include "common/dt_validation.h"

void raw_speed_init(raw_speed_estimator_t *e) {
    e->last_range_m = 0.0;
    e->last_ts.t_s = 0.0;
    e->has_prev = false;
    e->speed.value_mps = 0.0;
    e->speed.is_stale = true; /* nothing computed yet */
}

void raw_speed_on_new_range(raw_speed_estimator_t *e, double range_m, timestamp_t ts) {
    if (!e->has_prev) {
        e->last_range_m = range_m;
        e->last_ts = ts;
        e->has_prev = true;
        e->speed.is_stale = true; /* no prior sample to difference against */
        return;
    }
    double dt = dt_between(e->last_ts, ts);
    if (dt_is_valid(dt)) {
        e->speed.value_mps = (e->last_range_m - range_m) / dt;
        e->speed.is_stale = false;
    } else {
        e->speed.is_stale = true; /* hold previous value_mps, do not divide by bad dt */
    }
    e->last_range_m = range_m;
    e->last_ts = ts;
}

void raw_speed_mark_stale(raw_speed_estimator_t *e) {
    e->speed.is_stale = true; /* value_mps left unchanged (held) */
}
