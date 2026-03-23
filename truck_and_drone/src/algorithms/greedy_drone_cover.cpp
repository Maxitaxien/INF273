#include "algorithms/greedy_drone_cover.h"
#include "general/get_customer_positions.h"
#include <vector>

const int DRONES = 2;

Solution greedy_drone_cover(const Instance& inst, Solution base) {
    // To store new solution
    Solution new_solution;
    new_solution.truck_route.clear();
    new_solution.drones.resize(DRONES);

    // For storing temporary drone movements before recomputing indices
    std::vector<DroneCollection> copy(DRONES);

    const int lim = inst.lim;
    const int route_size = (int)(base.truck_route.size());
    int drone_counter = 0;

    int i = 0;
    while (i < route_size - 2) {
        const int a = base.truck_route[i];
        const int b = base.truck_route[i + 1];
        const int c = base.truck_route[i + 2];

        const long long drone_time = inst.drone_matrix[a][b] + inst.drone_matrix[b][c];
        const long long truck_time = inst.truck_matrix[a][c];
        const long long effective_drone_time = std::max(drone_time, truck_time);

        new_solution.truck_route.push_back(a);

        if (effective_drone_time < inst.truck_matrix[a][b] + inst.truck_matrix[b][c] && effective_drone_time < lim) {
            const int d = drone_counter % DRONES;
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
    while (i < route_size) {
        new_solution.truck_route.push_back(base.truck_route[i]);
        i++;
    }

    const std::unordered_map<int, int> customer_positions = get_customer_positions(new_solution);

    // Fix indices for drone trips
    for (int d = 0; d < DRONES; d++) {
        const int trip_count = (int)(copy[d].launch_indices.size());
        for (int trip_idx = 0; trip_idx < trip_count; ++trip_idx) {
            const int launch_idx = customer_positions.at(copy[d].launch_indices[trip_idx]);
            const int deliver = copy[d].deliver_nodes[trip_idx];
            const int land_idx = customer_positions.at(copy[d].land_indices[trip_idx]);

            new_solution.drones[d].launch_indices.push_back(launch_idx);
            new_solution.drones[d].deliver_nodes.push_back(deliver);
            new_solution.drones[d].land_indices.push_back(land_idx);
        }
    }

    return new_solution;
}

