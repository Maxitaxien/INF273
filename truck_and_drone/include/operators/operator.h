#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"
#include <functional>
#include <vector>

using Operator = std::function<bool(const Instance &, Solution &)>;

using FullNeighbourhoodOperator = std::function<std::vector<Solution>(const Instance &, const Solution &)>;

/**
 * Generates all feasible one reinsert options.
 * Too slow for practical use.
 */
std::vector<Solution> one_reinsert_operator(const Instance &instance, const Solution &sol);

/**
 * Picks a random remove and reinsert candidate.
 */
bool one_reinsert_random(const Instance &instance, Solution &sol);

/**
 * Generate a one_reinsert candidate by first randomly selecting the candidate to remove.
 * Then evaluate all insertion candidates, greedily pick the best
 */
bool one_reinsert_greedy(const Instance &instance, Solution &sol);

/**
 * Then, evaluate all substitutions within the truck route for both drones.
 * Perform the best.
 */
bool substitute_truck_delivery_greedy(const Instance &instance, Solution &sol);

/**
 * Perform 2-opt on random indexes except from depot
 *
 * @return Non-meaningfull bool to conform - should always be valid move
 */
bool two_opt_random(const Instance &inst, Solution &sol);