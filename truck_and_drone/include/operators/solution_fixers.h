#pragma once

#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Assigns the best feasible launch/land pair to a newly inserted drone delivery.
 *
 * The truck route stays fixed. The assignment is chosen with the same timing model
 * as the feasibility checker, so delayed launches and waiting at reconnection are
 * handled consistently.
 */
std::pair<bool, Solution> greedy_assign_launch_and_land(
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
