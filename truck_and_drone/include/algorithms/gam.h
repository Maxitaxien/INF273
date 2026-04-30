#pragma once
#include "datahandling/instance.h"
#include "operators/operator.h"
#include "verification/solution.h"
#include <string>
#include <vector>
#include "algorithms/gamstructs.h"

struct GAMConfig
{
    double phase_one_fraction = 1.0;
    std::vector<std::string> phase_one_operator_names;
    std::vector<std::string> phase_two_operator_names;
};

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
    int time_limit_s,
    const GAMConfig &config,
    const std::vector<double> &initial_weights);

GAMResult general_adaptive_metaheuristic(
    const Instance &instance,
    Solution initial,
    const std::vector<NamedOperator> &ops,
    const std::vector<double> &initial_weights = {});
