#pragma once

#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Relocate a short consecutive truck segment to a new position.
 *
 * This is a classic intensification move for routing problems and is usually
 * cheaper than 3-opt while still fixing many route-order defects. A good first
 * implementation should support segment lengths 1..3.
 */
bool or_opt_segment_relocate(
    const Instance &inst,
    Solution &sol,
    int start_idx,
    int segment_length,
    int insert_after_idx);
