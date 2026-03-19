#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"
#include <unordered_map>
#include <utility>
#include <vector>

using Flight = std::pair<int, int>;
using P = std::unordered_map<int, std::vector<Flight>>;

/**
 * Builds map of set P_j = {(i, s) | i and s are truck-route nodes and
 * d(i, j) + d(j, s) <= L}.
 */
P build_p(const Instance &inst, const Solution &curr_sol);

/**
 * Randomly reassigns the customers currently served by drones.
 */
std::pair<long long, Solution> drone_planner(
    const Instance &inst,
    const Solution &curr_sol);
