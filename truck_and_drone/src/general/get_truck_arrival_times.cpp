#include "verification/solution.h"
#include "datahandling/instance.h"
#include <vector>
#include <iostream>


std::vector<long long> get_truck_arrival_times(
    const Instance& instance,
    const Solution& solution,
    std::vector<long long>& drone_available,
    long long& total_drone_arrival      // output parameter
) {
    int m = static_cast<int>(solution.truck_route.size());
    std::vector<long long> truck_arrival(m, 0);
    std::vector<long long> truck_departure(m, 0);

    drone_available.assign(solution.drones.size(), 0);
    total_drone_arrival = 0;                         // initialise
    long long latest_drone_return_so_far = 0;

    // Precompute which drones land at which truck index
    std::vector<std::vector<std::pair<int,int>>> drone_returns_at(m);
    for (int d = 0; d < static_cast<int>(solution.drones.size()); ++d) {
        const DroneCollection& c = solution.drones[d];
        for (int t = 0; t < static_cast<int>(c.launch_indices.size()); ++t) {
            if (c.land_indices[t] >= 0 && c.land_indices[t] < m) {
                drone_returns_at[c.land_indices[t]].emplace_back(d, t);
            } else {
                std::cerr << "Skipping drone landing at invalid truck index "
                          << c.land_indices[t] << " (truck route size: " << m << ")\n";
            }
        }
    }

    for (int i = 1; i < m; ++i) {
        int prev = solution.truck_route[i-1];
        int curr = solution.truck_route[i];

        truck_arrival[i] = truck_departure[i-1] + instance.truck_matrix[prev][curr];

        // Process drones landing at this stop
        for (auto [d, t] : drone_returns_at[i]) {
            const DroneCollection& c = solution.drones[d];

            int launch_idx   = c.launch_indices[t];
            int launch_node  = solution.truck_route[launch_idx];
            int deliver_node = c.deliver_nodes[t];
            int land_node    = curr;

            long long out_time  = instance.drone_matrix[launch_node][deliver_node];
            long long back_time = instance.drone_matrix[deliver_node][land_node];

            long long launch_time   = std::max(truck_arrival[launch_idx], drone_available[d]);
            long long drone_arrival = launch_time + out_time;
            long long drone_return  = drone_arrival + back_time;

            // Accumulate drone arrival times into total_drone_arrival
            total_drone_arrival += drone_arrival;

            drone_available[d] = drone_return;
            latest_drone_return_so_far = std::max(latest_drone_return_so_far, drone_return);
        }

        truck_departure[i] = std::max(truck_arrival[i], latest_drone_return_so_far);
    }

    return truck_arrival;
}