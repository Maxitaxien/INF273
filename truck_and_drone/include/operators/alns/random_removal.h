#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Removes n random elements from the solution.
 */
bool random_removal(const Instance &inst, Solution &sol, int n);