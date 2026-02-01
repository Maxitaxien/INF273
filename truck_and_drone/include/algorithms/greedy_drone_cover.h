#pragma once
#include "verification/solution.h"
#include "datahandling/instance.h"

/**
 * Given values from distance matrix and drone flight range limit,
 * gives whether it is feasible to cover this node with the drone.
 * 
 * @param lim drone flight range limit
 * @param ab distance from launch to customer
 * @param bc distance from customer to land
 * @return whether the drone can cover this delivery
 */
bool is_feasible(int lim, long long ab, long long bc);

/**
 * Given a truck tour, tries to assign a drone for covering at every step
 * So for instance, say the truck at some points visits: 1 -> 2 -> 3
 * We will try to assign a drone: 1 -> 2 -> 3 (if feasible and good). The truck route can then be reduced to: 1 -> 3.
 * This will only use one of the drones
 * 
 * @param Instance A problem instance for truck and drone
 * 
 * @param Solution A valid solution for truck and drone, generated from the instance
 * @attention Solution should consist of only a truck tour, with no drone assignments yet
 * 
 * @return New solution with possible greedy drone coverings.
 * 
 */
Solution greedy_drone_cover(const Instance& problem_instance, Solution truck_solution);