#pragma once
#include "verification/solution.h"
#include "verification/feasibility_check.h"
#include "general/roulette_wheel_selection.h"
#include "operators/operator.h"

#include <vector>

/**
 * Escape algorithm: 
 * 
 * Applies a short random walk over the current operator set, but only keeps
 * feasible intermediate solutions.
 */
Solution gam_escape_algorithm(
    const Instance& inst, 
    Solution incumbent, 
    const std::vector<NamedOperator> &ops, 
    const std::vector<double> &selection_weights,
    int amnt_iter
);
