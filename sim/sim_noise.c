#include "sim/sim_noise.h"
#include <math.h>

static uint32_t xorshift32(uint32_t *state) {
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

static double uniform01(sim_noise_t *n) {
    return (double)xorshift32(&n->state) / 4294967296.0;
}

void sim_noise_seed(sim_noise_t *n, uint32_t seed) {
    n->state = seed ? seed : 1u; /* xorshift requires nonzero state */
}

double sim_noise_gaussian(sim_noise_t *n, double mean, double stddev) {
    double u1 = uniform01(n);
    double u2 = uniform01(n);
    if (u1 < 1e-12) u1 = 1e-12;
    double z = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2); /* Box-Muller */
    return mean + stddev * z;
}
