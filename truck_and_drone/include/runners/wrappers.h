#pragma once
#include <functional>
#include "datahandling/instance.h"
#include "verification/solution.h"

using Algorithm = std::function<Solution(
    const Instance &,
    Solution,
    std::function<bool(const Instance &, Solution &)>)>;

Solution blind_random_wrapper(
    const Instance &instance,
    Solution initial,
    std::function<bool(const Instance &, Solution &)> op);

Solution sa_wrapper(
    const Instance &instance,
    Solution initial,
    std::function<bool(const Instance &, Solution &)> op);

Solution local_search_wrapper(
    const Instance &instance,
    Solution initial,
    std::function<bool(const Instance &, Solution &)> op);

Solution nearest_neighbour_wrapper(
    const Instance &instance,
    Solution initial,
    std::function<bool(const Instance &, Solution &)> op);

Solution construction_wrapper(
    const Instance &instance,
    Solution initial,
    std::function<bool(const Instance &, Solution &)> op);