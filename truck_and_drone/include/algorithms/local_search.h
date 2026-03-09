#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"
#include "operators/operator.h"
#include <functional>

/**
 * Runs a best-first local search
 */
Solution local_search(
    const Instance &instance,
    const Solution &initial,
    Operator op);