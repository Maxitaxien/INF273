#pragma once
#include "verification/solution.h"
#include "verification/feasibility_check.h"
#include "general/roulette_wheel_selection.h"
#include "operators/operator.h"

#include <vector>

struct GAMEscapeResult
{
    Solution incumbent;
    Solution best_seen;
    bool found_new_best = false;
};

/**
 * Escape algorithm:
 *
 * Applies a short random walk over the current operator set, keeping the final
 * feasible incumbent while also tracking the best solution seen during the
 * walk.
 */
GAMEscapeResult gam_escape_algorithm(
    const Instance& inst, 
    Solution incumbent, 
    const std::vector<NamedOperator> &ops, 
    const std::vector<double> &selection_weights,
    int amnt_iter
);
