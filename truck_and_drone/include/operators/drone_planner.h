#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"
#include <unordered_map>
#include <utility>
#include <vector>

using Flight = std::pair<int, int>;
using P = std::unordered_map<int, std::vector<Flight>>;

/**
 * Randomly reassigns the customers currently served by drones.
 */
std::pair<long long, Solution> drone_planner(
    const Instance &inst,
    const Solution &curr_sol);

/**
 * Budgeted planner variant for use inside fast neighbourhood search.
 *
 * `iterations` limits the number of randomized rebuilds.
 * `max_flights_per_customer` keeps only the cheapest candidate flights per
 * customer before timing checks. Use 0 to keep all candidates.
 */
std::pair<long long, Solution> drone_planner(
    const Instance &inst,
    const Solution &curr_sol,
    int iterations,
    int max_flights_per_customer);

/**
 * Budgeted planner that can optionally rebuild a single drone route.
 *
 * Set `drone_to_replan` to -1 to rebuild every drone route.
 */
std::pair<long long, Solution> drone_planner(
    const Instance &inst,
    const Solution &curr_sol,
    int iterations,
    int max_flights_per_customer,
    int drone_to_replan);
