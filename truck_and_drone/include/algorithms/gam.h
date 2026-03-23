#pragma once
#include "datahandling/instance.h"
#include "operators/operator.h"
#include "verification/solution.h"

/**
 * Scaffold for the General Adaptive Metaheuristic (GAM).
 *
 * The operator argument can be any regular operator, ALNS composite operator,
 * or weighted selector over a mix of both.
 */
Solution general_adaptive_metaheuristic(
    const Instance &instance,
    Solution initial,
    Operator op);
