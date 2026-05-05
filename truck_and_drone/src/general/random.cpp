#include "general/random.h"
#include <random>
#include <algorithm>

thread_local std::mt19937 gen(DEFAULT_RANDOM_SEED);

void seed_random(unsigned int seed) {
    gen.seed(seed);
}

int rand_int(int a, int b) {
    std::uniform_int_distribution<> dist(a, b);
    return dist(gen);
}

double rand_double(double a, double b) {
    std::uniform_real_distribution<> dist(a, b);
    return dist(gen);
}
