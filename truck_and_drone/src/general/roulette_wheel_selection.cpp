#include "general/roulette_wheel_selection.h"
#include "general/random.h"
#include <vector>

namespace
{
std::vector<double> build_exponential_weights(int n, double decay)
{
    if (decay <= 0.0 || decay >= 1.0)
    {
        decay = 0.28;
    }

    std::vector<double> weights;
    weights.reserve(n);

    double weight = 1.0;
    for (int i = 0; i < n; ++i)
    {
        weights.push_back(weight);
        weight *= decay;
    }

    return weights;
}
}

int roulette_wheel_selection_uniform(int n)
{
    if (n <= 0) return 0;
    double uniform_fitness = 1.0 / n;
    std::vector<double> fitness(n, uniform_fitness);
    return roulette_wheel_selection(fitness);
}

int roulette_wheel_selection(const std::vector<double>& fitness)
{
    if (fitness.empty()) return 0;

    double total_fitness = 0.0;
    for (double f : fitness)
    {
        if (f > 0.0)
        {
            total_fitness += f;
        }
    }

    if (total_fitness <= 0.0)
    {
        return roulette_wheel_selection_uniform(fitness.size());
    }

    double r = rand_double(0.0, total_fitness);

    double cumulative = 0.0;
    for (int i = 0; i < fitness.size(); ++i)
    {
        if (fitness[i] <= 0.0)
        {
            continue;
        }

        cumulative += fitness[i];
        if (r <= cumulative)
        {
            return i;
        }
    }

    return fitness.size() - 1;
}

int roulette_wheel_selection_exponential(int n, double decay)
{
    if (n <= 0) return 0;
    return roulette_wheel_selection(build_exponential_weights(n, decay));
}

