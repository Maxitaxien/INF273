#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"
#include "operators/operator.h"
#include <functional>

/**
 * Runs a best-first local search
 */
Solution local_search(
    const Instance& instance,
    const Solution& initial,
    std::function<bool(const Instance&, Solution&)> op,
    std::function<long long(const Instance&, const Solution&)> objective
);