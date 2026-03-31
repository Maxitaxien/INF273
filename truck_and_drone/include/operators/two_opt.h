#pragma once

#include "verification/solution.h"
#include "datahandling/instance.h"

/**
 * 2-opt pseudocode:
 *
 * procedure 2optSwap(route, v1, v2) {
 *  1. take route[start] to route[v1] and add them in order to new_route
 *  2. take route[v1+1] to route[v2] and add them in reverse order to new_route
 *  3. take route[v2+1] to route[start] and add them in order to new_route
 *  return new_route;
 *
 * This operator mutates the truck route and, if needed, repairs drone
 * assignments before returning.
 */
bool two_opt(const Instance &inst, Solution &solution, int first, int second);

/**
 * Greedy 2-opt with a truck-gain screen and full-objective evaluation.
 *
 * The best positive truck 2-opt edge exchange is tried, and the move is only
 * committed if the repaired full truck+drone objective improves.
 */
bool two_opt_greedy(const Instance &inst, Solution &solution);

/**
 * First-improvement 2-opt with a truck-gain screen and full-objective
 * evaluation.
 *
 * This stops at the first improving feasible reversal instead of scanning the
 * full neighbourhood for the best gain.
 */
bool two_opt_first_improvement(const Instance &inst, Solution &solution);
