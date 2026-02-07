#include "algorithms/helpers.h"
#include "verification/solution.h"
#include <unordered_map>

void recompute_indices(Solution& solution) {
    std::unordered_map<int,int> pos;
    for (int i = 0; i < solution.truck_route.size(); i++)
        pos[solution.truck_route[i]] = i;

    for (auto& drone_list : solution.drones) {
        for (auto& trip : drone_list) {
            trip.launch_index = pos[trip.launch_node];
            trip.land_index   = pos[trip.land_node];
        }
    }
}