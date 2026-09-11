#ifndef SIM_SIM_RANGE_SENSOR_H
#define SIM_SIM_RANGE_SENSOR_H

#include <stdbool.h>
#include "common/types.h"
#include "common/measurement.h"
#include "sim/sim_noise.h"

typedef struct {
    double period_s;         /* nominal sensor conversion period, e.g. 0.05 (20Hz) */
    double noise_stddev_m;
    double last_emit_t_s;    /* -1 before first sample */
    bool dropout_active;     /* test hook: force HAL_NO_NEW_DATA regardless of period */
    sim_noise_t noise;
} sim_range_sensor_t;

void sim_range_sensor_init(sim_range_sensor_t *s, double period_s, double noise_stddev_m, uint32_t seed);

/* Emits HAL_OK with a fresh noisy sample once per period_s of sim time
   elapsed since the last emission; HAL_NO_NEW_DATA otherwise or while a
   dropout is active. */
hal_status_t sim_range_sensor_sample(sim_range_sensor_t *s, double true_range_m, timestamp_t now, range_sample_t *out);

void sim_range_sensor_set_dropout(sim_range_sensor_t *s, bool active);

#endif /* SIM_SIM_RANGE_SENSOR_H */
