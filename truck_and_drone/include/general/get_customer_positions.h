#pragma once
#include "verification/solution.h"
#include <unordered_map>

/**
 * Get index of each customer within the truck route.
 */
std::unordered_map<int, int> get_customer_positions(const Solution &solution);
