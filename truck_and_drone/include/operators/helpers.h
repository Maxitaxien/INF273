#pragma once
#include "verification/solution.h"
#include "datahandling/instance.h"
#include <vector>
#include <set>

// TODO: Some of these functions probably should be put into separate files
// Work on refactoring this after bugs are ironed out

/**
 * Struct for drone delivery intervals for a single drone
 * Allows for easy overlap checks
 */
struct Interval
{
    int start, end;
    bool operator<(const Interval &other) const { return start < other.start || (start == other.start && end < other.end); }
};

/**
 * Checks if any of the drones have a landing index at the last index in the truck route.
 * Relevant if it is popped from the truck route, as we may have to fix the drone assignments in this case.
 * 
 * Returns a pair where each entry corresponds to whether the drone landed at the back index.
 */
std::pair<bool, bool> drone_landed_at_back(const Solution& solution);

/**
 * Sorts points in accordance to closeness, then returns the closest from the truck route
 * @param point: point to sort in relation to
 */
std::vector<int> sort_points(const Instance &instance, const Solution &solution, int point);

/**
 * Get time intervals the drone in question is occupied
 */
std::set<Interval> get_intervals(const Solution &solution, int drone);

/**
 * Check if a potential new launch -> land overlaps existing drone launch -> land sequence.
 * Then, it is guaranteed to be the case not to be valid to insert it.
 * 
 * Logic is c++ black magic but efficient
 */
bool overlaps(const std::set<Interval>& intervals, int pos1, int pos2);

/**
 * Check if a idx inserted into a truck route is contained in any drone flights
 * Then, this may potentially break their feasibility.
 * 
 * Logic is c++ black magic like overlaps().  
 * @return index of interval, which should correspond to index of drone flight
 */
int find_containing_interval_index(const std::set<Interval>& intervals, int pos);

/**
 * Get cumulative truck arrival times.
 * Can use this to take a difference to get the time the truck spends on getting to the node -
 * then we can take the max of this and the drone travel time to get effective drone travel time.
 */
std::vector<long long> get_truck_arrival_time_at_index(const Instance &instance, const Solution &solution);


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
 * If we're lucky
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