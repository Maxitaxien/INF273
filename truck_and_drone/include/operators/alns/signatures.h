#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"
#include <functional>
#include <string>

/**
 * Removes an element based on some rule,
 * or randomly.
 * 
 *
 * @param n The amount of times to apply the operation (increases neighbourhood size).
 * @return a pair containing a bool indicating if the removal was successfull and the removed elements.
 */
using RemovalHeuristic = std::function<std::pair<bool, std::vector<int>>(const Instance &, Solution &, int n)>;

/**
 * Inserts greedily (k=0) or with regret (k > 1).
 *
 * @param to_insert a vector of values to insert. Size corresponds to n-value used in removal heuristic
 * @return whether the insertion was successfull
 */
using InsertionHeuristic = std::function<bool(const Instance &, Solution &, std::vector<int> to_insert, int k)>;

struct NamedRemovalHeuristic
{
    std::string name;
    RemovalHeuristic op;
};

struct NamedInsertionHeuristic
{
    std::string name;
    InsertionHeuristic op;
};
