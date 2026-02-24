#pragma once
#include <functional>
#include "datahandling/instance.h"
#include "verification/solution.h"

using Algorithm = std::function<Solution(
    const Instance&,
    Solution,
    std::function<bool(const Instance&, Solution&)>,
    std::function<long long(const Instance&, const Solution&)>
)>;


Solution blind_random_wrapper(
    const Instance& instance,
    Solution initial,
    std::function<bool(const Instance&, Solution&)> op,
    std::function<long long(const Instance&, const Solution&)> objective
);

Solution sa_wrapper(
    const Instance& instance,
    Solution initial,
    std::function<bool(const Instance&, Solution&)> op,
    std::function<long long(const Instance&, const Solution&)> objective
);

Solution local_search_wrapper(
    const Instance& instance,
    Solution initial,
    std::function<bool(const Instance&, Solution&)> op,
    std::function<long long(const Instance&, const Solution&)> objective
);