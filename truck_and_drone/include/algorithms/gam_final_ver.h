#pragma once
#include "verification/solution.h"
#include "algorithms/gamstructs.h"
#include "operators/operator.h"

/**
 * Runs time-based.
 * Simplified acceptance criterion
 */
GAMResult general_adaptive_metaheuristic_final(
    const Instance &instance,
    Solution initial,
    const std::vector<NamedOperator> &ops,
    const std::vector<double> &initial_weights = {});