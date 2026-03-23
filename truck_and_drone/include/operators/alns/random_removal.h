#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"
#include <utility>
#include <vector>

/**
 * Removes n random elements from the solution.
 */
std::pair<bool, std::vector<int>> random_removal(const Instance &inst, Solution &sol, int n);
