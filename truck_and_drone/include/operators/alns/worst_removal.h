#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Removes the worst candidate n times.
 *
 * Uses weights (roulette wheel) to not always select the absolute worst candidate
 */
bool worst_removal(const Instance &inst, Solution &sol, int n);