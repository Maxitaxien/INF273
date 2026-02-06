#pragma once
#include<vector>

struct DroneTrip {
    int launch_node;     
    int delivery_node;  
    int land_node;      

    // Cached indices for fast feasibility checks
    int launch_index = -1;
    int land_index = -1;
};

struct Solution {
    // @brief A vector of ints representing the current truck route
    std::vector<int> truck_route;
    // @brief A single array of drone trips
    std::vector<std::vector<DroneTrip>> drones; 
};