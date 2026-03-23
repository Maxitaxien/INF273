#include "algorithms/gam.h"
#include "verification/objective_value.h"
#include "verification/feasibility_check.h"
#include "general/random.h"
#include <utility>

Solution general_adaptive_metaheuristic(
    const Instance &instance,
    Solution initial,
    Operator op)
{

    Solution incumbent = std::move(initial);
    long long incumbent_cost = objective_function_impl(instance, incumbent);
    Solution best = incumbent;
    long long best_cost = incumbent_cost;

    constexpr int max_iterations = 10000;
    int non_improving_iterations = 0;
    constexpr int stopping_condition = 50;

    for (int i = 0; i < max_iterations; i++) {
        // TODO: Escape condition
        if (non_improving_iterations >= stopping_condition) {
            non_improving_iterations = 0;
        }

        Solution neighbour = incumbent;

        if (!op(instance, neighbour)) {
            continue;
        }

        if (!master_check(instance, neighbour, false)) {
            continue;
        }
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
            non_improving_iterations++;
            double p; // TODO: acceptance criterion
            if (rand_double(0.0, 1.0) < p)
            {
                incumbent = neighbour;
                incumbent_cost = cost;
            }
        }



    }
    // TODO 5: Define the acceptance rule for moving from `current` to the
    // candidate (improving-only, threshold, SA-style, late acceptance, etc.).
    // TODO 7: If you want true adaptivity, maintain operator-level scores and
    // update the selector weights in segments instead of keeping them static.
    // TODO 8: Log iteration statistics so you can inspect which operators are
    // useful and when the search gets stuck.

    return best;
}
