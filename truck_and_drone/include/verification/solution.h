#pragma once
#include<vector>

struct DroneCollection {
    std::vector<int> launch_indices;    
    std::vector<int> deliver_nodes;
    std::vector<int> land_indices;    
};

struct Solution {
    // @brief A vector of ints representing the current truck route. Includes depot (0) at start, but implicitly ends at depot as well
    std::vector<int> truck_route;
    // @brief A single array of drone trips for each drone. Has indices within truck route for launch and land,  
    // and deliver nodes indicating the nodes delivered to which are not covered by truck. Both indices are 0-indexed.
    std::vector<DroneCollection> drones; 
};