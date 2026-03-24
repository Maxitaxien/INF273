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
 * This operator only mutates the truck route. The surrounding search
 * algorithm is expected to perform the final feasibility check.
 */
bool two_opt(const Instance &inst, Solution &solution, int first, int second);

/**
 * Greedy 2-opt on truck edges using the standard edge-exchange gain:
 *
 * gain = dist(a,b) + dist(c,d) - dist(a,c) - dist(b,d)
 *
 * where `(a,b)` and `(c,d)` are the two truck edges being replaced.
 * The best improving move is committed.
 */
bool two_opt_greedy(const Instance &inst, Solution &solution);
