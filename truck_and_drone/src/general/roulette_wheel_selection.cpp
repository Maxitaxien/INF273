#include "general/roulette_wheel_selection.h"
#include <vector>
#include <random>
int roulette_wheel_selection_uniform(int n) {
    if (n <= 0) return 0;
    double uniform_fitness = 1.0 / n;
    std::vector<double> fitness(n, uniform_fitness);
    return roulette_wheel_selection(fitness);
}

int roulette_wheel_selection(const std::vector<double>& fitness) {
    // Sum all fitness values
    double total_fitness = 0.0;
    for (double f : fitness)
        total_fitness += f;

    // Generate random number
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(0.0, total_fitness);

    double r = dist(gen);

    // 3. Find selected individual
    double cumulative = 0.0;
    for (int i = 0; i < fitness.size(); i++) {
        cumulative += fitness[i];
        if (r <= cumulative)
            return i;
    }

    return fitness.size() - 1; // fallback
}