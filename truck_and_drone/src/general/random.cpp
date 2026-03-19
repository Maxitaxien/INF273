#include "general/random.h"
#include <random>
#include <algorithm>

const int SEED = 42;
std::mt19937 gen(SEED);

int rand_int(int a, int b) {
    std::uniform_int_distribution<> dist(a, b);
    return dist(gen);
}

double rand_double(double a, double b) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(a, b);

    return dist(gen);
}