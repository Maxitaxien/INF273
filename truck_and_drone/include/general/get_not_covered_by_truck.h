#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"
#include <vector>

/**
 * Used when no drone assignments exist at all
 */
std::vector<int> get_not_covered_by_truck(int n, const Solution &sol);

/**
 * Used when we already have drone deliveries - faster
 */
std::vector<int> get_not_covered_by_truck(const Solution &sol);