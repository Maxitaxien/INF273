#include "runners/wrappers.h"
#include "algorithms/nearest_neighbour.h"
#include "algorithms/greedy_drone_cover.h"
#include "algorithms/blind_random_search.h"
#include "algorithms/simulated_annealing.h"
#include "algorithms/local_search.h"

Solution blind_random_wrapper(
    const Instance &instance,
    Solution initial,
    std::function<bool(const Instance &, Solution &)> op)
{
    return blind_random_search(instance, initial);
}

Solution sa_wrapper(
    const Instance &instance,
    Solution initial,
    std::function<bool(const Instance &, Solution &)> op)
{
    return simulated_annealing(instance, initial, op, 0.1);
}

Solution local_search_wrapper(
    const Instance &instance,
    Solution initial,
    std::function<bool(const Instance &, Solution &)> op)
{
    return local_search(instance, initial, op);
}

Solution nearest_neighbour_wrapper(
    const Instance &instance,
    Solution initial,
    std::function<bool(const Instance &, Solution &)> op)
{
    return nearest_neighbour(instance);
}

Solution construction_wrapper(
    const Instance &instance,
    Solution initial,
    std::function<bool(const Instance &, Solution &)> op)
{
    Solution nn = nearest_neighbour(instance);
    return greedy_drone_cover(instance, nn);
}