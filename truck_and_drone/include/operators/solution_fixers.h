#pragma once

#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Tries to assign a delivery to the given drone by launching at `idx` and
 * landing within a limited lookahead window. Timing is evaluated with the same
 * synchronized truck/drone model as the feasibility checker.
 */
std::pair<bool, Solution> assign_launch_and_land_n_lookahead(
    const Instance &instance,
    Solution &solution,
    int idx,
    int new_deliver,
    int drone,
    int look_ahead);

/**
 * Faster variant of the lookahead assigner for hot paths where the caller
 * already preserves structural validity of the solution.
 */
std::pair<bool, Solution> assign_launch_and_land_n_lookahead_assume_valid(
    const Instance &instance,
    Solution &solution,
    int idx,
    int new_deliver,
    int drone,
    int look_ahead);

/**
 * Assigns a feasible local launch/land pair to a newly inserted drone delivery.
 *
 * The truck route stays fixed. The search is bounded to a small vicinity around
 * promising truck stops so hot local operators stay cheap; heavier global
 * improvement is left to the drone planner.
 */
std::pair<bool, Solution> greedy_assign_launch_and_land(
    const Instance &instance,
    Solution &solution,
    int new_deliver,
    int drone);

/**
 * Faster variant of greedy drone assignment for callers that already operate
 * on a clean solution and do not need a full validity cleanup first.
 */
std::pair<bool, Solution> greedy_assign_launch_and_land_assume_valid(
    const Instance &instance,
    Solution &solution,
    int new_deliver,
    int drone);

/**
 * Repairs a single drone schedule after a truck-route mutation.
 */
Solution &fix_feasibility_for_drone(const Instance &instance, Solution &sol, int drone);

/**
 * Repairs the full solution after a route mutation.
 *
 * The truck route is kept fixed. The drone planner is tried first to globally
 * rebuild the drone schedule, and the legacy single-drone repair is only used
 * as a fallback.
 */
Solution fix_overall_feasibility(const Instance &instance, Solution &solution);

/**
 * Cheap validity normalizer for stale, duplicate, or unusable drone flights.
 * Invalid flights are removed and uncovered customers are appended back to the
 * truck route exactly once.
 */
Solution &simple_fix_validity(Solution &solution);
