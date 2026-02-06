#include "algorithms/greedy_drone_cover.h"
#include "algorithms/helpers.h"
#include <vector>

const int DRONES = 2;

Solution greedy_drone_cover(const Instance& inst, Solution base) {
    Solution new_solution;
    new_solution.truck_route.clear();
    new_solution.drones.resize(DRONES);

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
            // Assign to alternating drones
            int d = drone_counter % DRONES;
            drone_counter++;

            DroneTrip trip;
            trip.launch_node = a;
            trip.delivery_node = b;
            trip.land_node = c;

            new_solution.drones[d].push_back(trip);

            i += 2; // skip b
        } else {
            new_solution.truck_route.push_back(b);
            i += 2; // move forward normally
        }
    }

    // Add remaining nodes
    while (i < n) {
        new_solution.truck_route.push_back(base.truck_route[i]);
        i++;
    }

    // Recompute cached indices
    recompute_indices(new_solution);

    return new_solution;
}
