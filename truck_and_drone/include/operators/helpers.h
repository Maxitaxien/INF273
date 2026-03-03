#pragma once
#include "verification/solution.h"
#include "datahandling/instance.h"
#include <vector>
#include <set>
#include <unordered_map>


/**
 * Remove drone flight at back index.
 */
void remove_drone_flight(Solution& solution, int drone);

/**
 * Remove drone flight at given index.
 */
void remove_drone_flight(Solution& solution, int drone, int i);

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
std::vector<int> sort_by_distance_to_point(const Instance &instance, const Solution &solution, int point);

/**
 * Get index of each node within the truck route (getting their ordering)
 */
std::unordered_map<int, int> get_node_positions(const Solution& solution);
