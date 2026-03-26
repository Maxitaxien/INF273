#pragma once

#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Truck-only 3-opt reconnection for a fixed triple of breakpoints.
 *
 * This helper only rewires the truck route and keeps the best improving
 * classic 3-opt candidate.
 */
bool three_opt(const Instance &inst, Solution &sol, int first, int second, int third);
