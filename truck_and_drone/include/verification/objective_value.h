#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Main objective calculation.
 */
long long objective_function_impl(const Instance &instance, const Solution &solution);
