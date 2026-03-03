#pragma once

#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Assigns eligible launch/land pair to a newly inserted drone delivery
 * @param drone: either 0 or 1 depending on if this applies to first or second rone
 * @return a pair with a bool indicating if the operation was successfull. If true, returns modified solution.
 * If false, returns old solution unmodified.
 */
std::pair<bool, Solution> assign_launch_and_land(const Instance &instance, Solution &solution, int new_deliver, int drone);

/**
 * Reassigns drone intervals for the infeasible drone to restore feasibility.
 * Situation: Two nodes in the truck have been swapped. 
 * The drone has a flight which overlaps with this node.
 * 
 */
Solution& fix_feasibility_for_drone(const Instance& instance, Solution& sol, int drone);

/**
 * To be used when both drones are infeasible, to handle both at the same time.
 */
Solution  fix_overall_feasibility(const Instance& instance, Solution& solution);

/**
 * We get invalidity through 1-insert if a drone was landing at the final index, and we remove it from the truck route.
 * Simple fix method: Just add it back in again.
 */
Solution simple_fix_validity(Solution& solution);

/**
 * More sophisticated validity fixer method.
 * Remove the now invalid launch -> deliver -> land sequence, then use assign_launch_and_land method on the missing delivery.
 */
Solution fix_validity(const Instance& instance, Solution& solution, int drone);