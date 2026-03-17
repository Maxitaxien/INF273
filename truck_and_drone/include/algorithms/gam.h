#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"
#include "operators/alns/signatures.h"
#include "runners/algorithms.h"
#include <vector>

/**
 * An implementation of the general adaptive
 * metaheuristic framework using alns.
 *
 * Essentially the same as ALNS but also features
 * the possibility of applying an escape algorithm
 */
Solution general_adaptive_metaheuristic_alns(
    std::vector<RemovalHeuristic> remove,
    std::vector<InsertionHeuristic> insert,
    Algorithm escape_algorithm,
    int escape_condition);

