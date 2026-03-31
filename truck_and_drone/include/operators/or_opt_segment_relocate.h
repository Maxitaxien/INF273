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

/**
 * First-improvement combined-vector Or-opt relocation.
 *
 * Scans short contiguous customer blocks and commits the first feasible move
 * that improves the full objective after repair.
 */
bool or_opt_segment_relocate_first_improvement(
    const Instance &inst,
    Solution &sol);

/**
 * Random combined-vector Or-opt relocation.
 *
 * Samples a short contiguous customer block and a random destination, repairs
 * with the drone planner if needed, and returns the changed feasible neighbour.
 */
bool or_opt_segment_relocate_random(
    const Instance &inst,
    Solution &sol);
