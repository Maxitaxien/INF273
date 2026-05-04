#pragma once
#include "datahandling/instance.h"
#include "operators/operator.h"
#include "verification/solution.h"
#include <string>
#include <vector>
#include "algorithms/gamstructs.h"

enum class GAMAcceptanceMode
{
    SimulatedAnnealing,
    BestRelativeRRT,
};

enum class GAMEscapeMode
{
    LegacyGAM,
    ExchangeKLarge,
};

struct GAMConfig
{
    double phase_one_fraction = 1.0;
    double allowed_deviation_fraction = 0.20;
    bool reset_acceptance_each_phase = true;
    GAMAcceptanceMode acceptance_mode = GAMAcceptanceMode::SimulatedAnnealing;
    GAMEscapeMode escape_mode = GAMEscapeMode::ExchangeKLarge;
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
