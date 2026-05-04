#pragma once

#include "datahandling/instance.h"
#include "verification/solution.h"

bool exact_segment_reopt(const Instance &inst, Solution &sol, int start_idx, int k);

bool exact_segment_reopt_random(
    const Instance &inst,
    Solution &sol,
    int k);