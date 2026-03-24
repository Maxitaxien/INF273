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
 */
bool two_opt(const Instance &inst, Solution &solution, int first, int second);
