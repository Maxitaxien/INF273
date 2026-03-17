#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"
#include "operators/alns/random_removal.h"
#include "operators/alns/worst_removal.h"
#include <functional>
#include <map>
#include <string>

/**
 * Removes an element based on some rule,
 * or randomly
 *
 * @param n: the amount of times to apply the operation (increases neighbourhood size)
 */
using RemovalHeuristic = std::function<bool(const Instance &, Solution &, int n)>;

/**
 * Inserts greedily (k=0) or with regret (k > 1)
 *
 * @param n: the amount of times to apply the operation (increases neighbourhood size)
 */
using InsertionHeuristic = std::function<bool(const Instance &, Solution &, int n, int k)>;
