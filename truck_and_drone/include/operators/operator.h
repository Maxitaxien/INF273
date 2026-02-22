#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"
#include <functional>
#include <vector>

using Operator = std::function<bool(const Instance&, const Solution&)>;

using FullNeighbourhoodOperator = std::function<std::vector<Solution>(const Instance&, const Solution&)>;

/**
 * Generates all feasible one reinsert options.
 * Too slow for practical use.
 */
std::vector<Solution> one_reinsert_operator(const Instance& instance, const Solution& sol);

bool one_reinsert_random(const Instance& instance, Solution& sol);