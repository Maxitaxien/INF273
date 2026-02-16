#include "algorithms/greedy_drone_cover.h"
#include <vector>
#include <unordered_map>

const int DRONES = 2;

Solution greedy_drone_cover(const Instance& inst, Solution base) {
    // To store new solution
    Solution new_solution;
    new_solution.truck_route.clear();
    new_solution.drones.resize(DRONES);

    // For storing temporary drone movements before recomputing indices
    std::vector<DroneCollection> copy;
    copy.resize(DRONES);

    int lim = inst.lim;
    int n = base.truck_route.size();
    int drone_counter = 0;

    int i = 0;
    while (i < n - 2) {
        int a = base.truck_route[i];
        int b = base.truck_route[i + 1];
        int c = base.truck_route[i + 2];

        long long drone_time = inst.drone_matrix[a][b] + inst.drone_matrix[b][c];
        long long truck_time = inst.truck_matrix[a][c];
        long long effective_drone_time = std::max(drone_time, truck_time);

        new_solution.truck_route.push_back(a);

        if (effective_drone_time < inst.truck_matrix[a][b] + inst.truck_matrix[b][c] && effective_drone_time < lim) {
            int d = drone_counter % DRONES;
            drone_counter++;

            // Add concrete nodes first, need to adjust indices later anyway
            DroneCollection& drone_collection = copy[d];
            drone_collection.launch_indices.push_back(a); // actually represents node      
            drone_collection.deliver_nodes.push_back(b);
            drone_collection.land_indices.push_back(c);

            i += 2; 
        } else {
            new_solution.truck_route.push_back(b);
            i += 2; 
        }
    }

    // Add remaining nodes
    while (i < n) {
        new_solution.truck_route.push_back(base.truck_route[i]);
        i++;
    }

    // get indices for nodes
    std::unordered_map<int, int> node_to_idx;
    for (int i = 0; i < new_solution.truck_route.size(); i++) {
        node_to_idx[new_solution.truck_route[i]] = i;
    }

    // Fix indices for drone trips
    for (int d = 0; d < DRONES; d++) {
        for (int i = 0; i < copy[d].launch_indices.size(); i++) {
            int launch_idx = node_to_idx[copy[d].launch_indices[i]];
            int deliver = copy[d].deliver_nodes[i];
            int land_idx = node_to_idx[copy[d].land_indices[i]];

            new_solution.drones[d].launch_indices.push_back(launch_idx);
            new_solution.drones[d].deliver_nodes.push_back(deliver);
            new_solution.drones[d].land_indices.push_back(land_idx);
        } 
    }

    return new_solution;
}
