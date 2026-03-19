#pragma once

#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Tries to look ahead n steps in the truck route, or until the drone's next delivery (whichever is more restrictive)
 * Test all launch -> deliver -> land combinations that are within flight range.
 */
std::pair<bool, Solution> assign_launch_and_land_n_lookahead(const Instance &instance, Solution &solution, int idx, int new_deliver, int drone, int look_ahead);

/**
 * Assigns eligible launch/land pair to a newly inserted drone delivery
 * @param drone: either 0 or 1 depending on if this applies to first or second rone
 * @return a pair with a bool indicating if the operation was successfull. If true, returns modified solution.
 * If false, returns old solution unmodified.
 */
std::pair<bool, Solution> greedy_assign_launch_and_land(const Instance &instance, Solution &solution, int new_deliver, int drone);

/**
 * Reassigns drone intervals for the infeasible drone to restore feasibility.
 * Situation: Two nodes in the truck have been swapped.
 * The drone has a flight which overlaps with this node.
 *
 */
Solution &fix_feasibility_for_drone(const Instance &instance, Solution &sol, int drone);

/**
 * A potentially faster individual drone feasibility fixer.
 * Should be tested in practice
 */
Solution &fix_feasibility_for_drone_alternative(const Instance &instance,
                                                Solution &sol, int drone);

/**
 * Repairs the full solution after a route mutation.
 *
 * The drone planner is tried first. If it fails to produce a feasible solution,
 * the legacy per-drone fixers are used as a fallback.
 */
Solution fix_overall_feasibility(const Instance &instance, Solution &solution);

/**
 * Cheap validity normalizer for drone flights with stale or unusable truck indices.
 * Invalid flights are removed and their delivered customers are appended back to the truck route.
 */
Solution simple_fix_validity(Solution &solution);

/**
 * More sophisticated validity fixer method.
 * Remove the now invalid launch -> deliver -> land sequence, then use greedy_assign_launch_and_land method on the missing delivery.
 */
Solution fix_validity(const Instance &instance, Solution &solution, int drone);

