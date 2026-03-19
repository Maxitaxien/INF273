#include "runners/wrappers.h"
#include "algorithms/blind_random_search.h"
#include "algorithms/greedy_drone_cover.h"
#include "algorithms/local_search.h"
#include "algorithms/nearest_neighbour.h"
#include "algorithms/simulated_annealing.h"
#include "operators/operator.h"
#include "verification/feasibility_check.h"

Solution blind_random_wrapper(
    const Instance &instance,
    Solution initial,
    Operator op)
{
    return blind_random_search(instance, initial);
}

Solution sa_wrapper(
    const Instance &instance,
    Solution initial,
    Operator op)
{
    return simulated_annealing(instance, initial, op, 0.1);
}

Solution local_search_wrapper(
    const Instance &instance,
    Solution initial,
    Operator op)
{
    return local_search(instance, initial, op);
}

Solution nearest_neighbour_wrapper(
    const Instance &instance,
    Solution initial,
    Operator op)
{
    return nearest_neighbour(instance);
}

Solution construction_wrapper(
    const Instance &instance,
    Solution initial,
    Operator op)
{
    Solution nn = nearest_neighbour(instance);
    return greedy_drone_cover(instance, nn);
}

Solution random_escape_wrapper(
    const Instance &instance,
    Solution current,
    Operator op)
{
    constexpr int escape_steps = 10;

    for (int i = 0; i < escape_steps; ++i)
    {
        Solution candidate = current;
        if (!op(instance, candidate))
        {
            continue;
        }

        if (!master_check(instance, candidate, false))
        {
            continue;
        }

        current = candidate;
    }

    return current;
}
