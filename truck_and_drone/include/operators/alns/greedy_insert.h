#pragma once
#include "verification/solution.h"
#include "datahandling/instance.h"

/**
 * Evaluates all insert positions.
 *
 * Picks greedily.
 *
 * Instead of deterministically picking best at each step, use roulette wheel
 * to introduce some variance.
 *
 * @param k always 0 (given as dummy to match signature, used in regret-k)
 */
bool greedy_insert(const Instance &inst, Solution &sol, int n, int k);