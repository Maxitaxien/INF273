#include "runners/wrappers.h"
#include "algorithms/blind_random_search.h"
#include "algorithms/greedy_drone_cover.h"
#include "algorithms/local_search.h"
#include "algorithms/nearest_neighbour.h"
#include "algorithms/simulated_annealing.h"
#include "verification/feasibility_check.h"

namespace
{
bool same_solution(const Solution &lhs, const Solution &rhs)
{
    if (lhs.truck_route != rhs.truck_route || lhs.drones.size() != rhs.drones.size())
    {
        return false;
    }

    for (int drone = 0; drone < (int)(lhs.drones.size()); ++drone)
    {
        const DroneCollection &lhs_collection = lhs.drones[drone];
        const DroneCollection &rhs_collection = rhs.drones[drone];
        if (lhs_collection.launch_indices != rhs_collection.launch_indices ||
            lhs_collection.deliver_nodes != rhs_collection.deliver_nodes ||
            lhs_collection.land_indices != rhs_collection.land_indices)
        {
            return false;
        }
    }

    return true;
}
}

Solution blind_random_wrapper(
    const Instance &instance,
    Solution initial,
    Operator op)
{
    (void)op;
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
    (void)initial;
    (void)op;
    return nearest_neighbour(instance);
}

Solution construction_wrapper(
    const Instance &instance,
    Solution initial,
    Operator op)
{
    (void)initial;
    (void)op;
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

        if (same_solution(candidate, current))
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
