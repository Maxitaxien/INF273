#include "algorithms/simulated_annealing.h"
#include "algorithms/random_valid_solution.h"
#include "verification/solution.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include "operators/operator.h"
#include "datahandling/instance.h"
#include "general/random.h"
#include <numeric>

// Warmup: return average delta, update incumbent & best properly
double warmup_phase(
    const Instance &instance,
    Solution &incumbent,
    long long &incumbent_cost,
    Solution &best,
    long long &best_cost,
    Operator op,
    int warmup_amount)
{
    std::vector<long long> delta_ws;
    delta_ws.reserve(warmup_amount);

    for (int w = 0; w < warmup_amount; w++)
    {
        Solution neighbour = incumbent; // try move on copy
        if (!op(instance, neighbour))
            continue;

        if (same_solution(neighbour, incumbent))
            continue;

        if (!master_check(instance, neighbour, false))
            continue;
        long long cost = objective_function_impl(instance, neighbour);

        long long delta_e = cost - incumbent_cost;

        if (delta_e < 0)
        {
            incumbent = neighbour;
            incumbent_cost = cost;
            if (cost < best_cost)
            {
                best = neighbour;
                best_cost = cost;
            }
        }
        else
        {
            if (rand_double(0.0, 1.0) < 0.8)
            {
                incumbent = neighbour;
                incumbent_cost = cost;
            }
            delta_ws.push_back(delta_e);
        }
    }

    if (delta_ws.empty())
        return 1.0; // prevent division by zero
    return std::accumulate(delta_ws.begin(), delta_ws.end(), 0.0) / delta_ws.size();
}

Solution simulated_annealing(
    const Instance &instance,
    Solution initial,
    Operator op,
    double TF)
{
    Solution incumbent = initial;
    Solution best = initial;
    long long incumbent_cost = objective_function_impl(instance, incumbent);
    long long best_cost = incumbent_cost;

    int warmup_amount = 100;
    double delta_avg = warmup_phase(instance, incumbent, incumbent_cost, best, best_cost, op, warmup_amount);

    int amnt_iterations = 10000 - warmup_amount;
    double T0 = -delta_avg / std::log(0.8);
    double alpha = pow(TF / T0, 1.0 / amnt_iterations);
    double T = T0;

    for (int i = 0; i < amnt_iterations; i++)
    {
        Solution neighbour = incumbent;
        if (!op(instance, neighbour))
            continue;

        if (same_solution(neighbour, incumbent))
            continue;

        if (!master_check(instance, neighbour, false))
            continue;
        long long cost = objective_function_impl(instance, neighbour);

        long long delta_e = cost - incumbent_cost;

        if (delta_e < 0)
        {
            incumbent = neighbour;
            incumbent_cost = cost;
            if (cost < best_cost)
            {
                best = neighbour;
                best_cost = cost;
            }
        }
        else
        {
            double p = exp(-delta_e / T);
            if (rand_double(0.0, 1.0) < p)
            {
                incumbent = neighbour;
                incumbent_cost = cost;
            }
        }

        T *= alpha;
    }

    return best;
}
