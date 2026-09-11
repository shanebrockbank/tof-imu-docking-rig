#ifndef SIM_SIM_NOISE_H
#define SIM_SIM_NOISE_H

#include <stdint.h>

typedef struct {
    uint32_t state;
} sim_noise_t;

void sim_noise_seed(sim_noise_t *n, uint32_t seed);
double sim_noise_gaussian(sim_noise_t *n, double mean, double stddev);

#endif /* SIM_SIM_NOISE_H */
