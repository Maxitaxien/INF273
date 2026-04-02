#pragma once
#include "verification/solution.h"
#include "operators/operator.h"

#include <vector>

struct GAMEscapeResult
{
    Solution incumbent;
    long long incumbent_cost = 0;
    Solution best_seen;
    long long best_seen_cost = 0;
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
