#include "sim/sim_range_sensor.h"

void sim_range_sensor_init(sim_range_sensor_t *s, double period_s, double noise_stddev_m, uint32_t seed) {
    s->period_s = period_s;
    s->noise_stddev_m = noise_stddev_m;
    s->last_emit_t_s = -1.0;
    s->dropout_active = false;
    sim_noise_seed(&s->noise, seed);
}

void sim_range_sensor_set_dropout(sim_range_sensor_t *s, bool active) {
    s->dropout_active = active;
}

hal_status_t sim_range_sensor_sample(sim_range_sensor_t *s, double true_range_m, timestamp_t now, range_sample_t *out) {
    if (s->dropout_active) {
        return HAL_NO_NEW_DATA;
    }
    bool due = (s->last_emit_t_s < 0.0) || (now.t_s - s->last_emit_t_s >= s->period_s);
    if (!due) {
        return HAL_NO_NEW_DATA;
    }
    s->last_emit_t_s = now.t_s;
    out->range_m = true_range_m + sim_noise_gaussian(&s->noise, 0.0, s->noise_stddev_m);
    if (out->range_m < 0.0) out->range_m = 0.0;
    out->ts = now;
    out->status = HAL_OK;
    return HAL_OK;
}
