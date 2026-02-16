#pragma once
#include<vector>

struct DroneCollection {
    std::vector<int> launch_indices;    
    std::vector<int> deliver_nodes;
    std::vector<int> land_indices;    
};

struct Solution {
    // @brief A vector of ints representing the current truck route
    std::vector<int> truck_route;
    // @brief A single array of drone trips for each drone
    std::vector<DroneCollection> drones; 
};