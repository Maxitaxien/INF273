#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Main objective calculation.
 */
long long objective_function_impl(const Instance &instance, const Solution &solution);

/**
 * Calculated objective function for truck only.
 * Useful as an indication of how good a modification to a truck route is.
 * May not fully represent the gain, but saves some time.
 */
long long objective_function_truck_only(const Instance &instance, const std::vector<int> &truck_route);