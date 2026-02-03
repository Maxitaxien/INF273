#pragma once
#include "verification/solution.h"
#include <random>

/**
 * Generates random solution guaranteed to be valid, but not necessarily feasible.
 * We need to have at least some nodes in the truck tour, as the drones leave from the truck.
 * Assuming there are two drones, for each a -> b pair in the truck tour, we can cover two other nodes c, d
 * Thefore, at least (n / 3) + 1 of the nodes must be covered by the truck, setting a lower bound for the random generation.
 * 
 * After the nodes in the truck route are set, distribute the remaining nodes randomly across drones. 
 * Then, for each drone, for each node they deliver to, pick a random launch spot among those available. 
 * Then pick a random land place from each node occuring after chosen land node. Update the map with these values. 
 */
Solution random_valid_solution(int n);
