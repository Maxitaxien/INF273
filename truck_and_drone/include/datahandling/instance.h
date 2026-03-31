#pragma once
#include <vector>

/**
 * Holds a loaded instance of the truck and drone problem.
 */
struct Instance {
    // @brief Amount of nodes to be visited 
    int n; 
    // @brief Amount of drones to be used - default 2.
    int m = 2;
    // @brief Drone flight limit until charge
    int lim;
    // @brief Matrix of travel costs for truck
    std::vector<std::vector<long long>> truck_matrix;
    // @brief Matrix of travel costs for drone
    std::vector<std::vector<long long>> drone_matrix;
    // @brief Flattened cache of pure-drone feasibility for (launch, customer, land).
    std::vector<unsigned char> pure_drone_feasible;
};
