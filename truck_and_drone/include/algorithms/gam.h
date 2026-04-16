#pragma once
#include "datahandling/instance.h"
#include "operators/operator.h"
#include "verification/solution.h"
#include <string>
#include <vector>
#include "algorithms/gamstructs.h"

/**
 * General Adaptive Metaheuristic (GAM).
 *
 * GAM owns the operator selection step itself so it can keep per-operator
 * scores, usages, and roulette-wheel weights.
 */
GAMResult general_adaptive_metaheuristic(
    const Instance &instance,
    Solution initial,
    const std::vector<NamedOperator> &ops,
    const std::vector<double> &initial_weights = {});
