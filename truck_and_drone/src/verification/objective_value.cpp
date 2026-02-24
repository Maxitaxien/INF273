#include "verification/objective_value.h"
#include "verification/feasibility_check.h"
#include <vector>
#include <algorithm>
#include <map>

long long calculate_total_waiting_time(
    const Instance& instance,
    const Solution& solution
) {
    long long total = 0;

    std::vector<int> route = solution.truck_route;
    route.push_back(0);
    int m = route.size();

    std::vector<long long> truck_arrival(m, 0);
    std::vector<long long> truck_departure(m, 0);

    std::vector<long long> drone_available(solution.drones.size(), 0);

    // Track the latest returning drone dynamically
    long long latest_drone_return_so_far = 0;

    // Precompute drone returns by truck index
    std::vector<std::vector<std::pair<int,int>>> drone_returns_at(m);
    for (int d = 0; d < solution.drones.size(); ++d) {
        const DroneCollection& c = solution.drones[d];
        int T = c.launch_indices.size();
        for (int t = 0; t < T; ++t) {
            if (c.deliver_nodes[t] != -1 && c.land_indices[t] != -1) {
                drone_returns_at[c.land_indices[t]].emplace_back(d, t);
            }
        }
    }

    for (int i = 1; i < m; ++i) {
        int prev = route[i - 1];
        int curr = route[i];

        truck_arrival[i] = truck_departure[i - 1] + instance.truck_matrix[prev][curr];

        // Update drones landing at this stop
        for (auto [d, t] : drone_returns_at[i]) {
            const DroneCollection& c = solution.drones[d];

            int launch_idx   = c.launch_indices[t];
            int launch_node  = route[launch_idx];
            int deliver_node = c.deliver_nodes[t];
            int land_node    = route[c.land_indices[t]];

            long long out_time  = instance.drone_matrix[launch_node][deliver_node];
            long long back_time = instance.drone_matrix[deliver_node][land_node];

            long long launch_time   = std::max(truck_arrival[launch_idx], drone_available[d]);
            long long drone_arrival = launch_time + out_time;
            long long drone_return  = drone_arrival + back_time;

            drone_available[d] = drone_return;
            total += drone_arrival;

            // Update the dynamic max
            latest_drone_return_so_far = std::max(latest_drone_return_so_far, drone_return);
        }

        // Truck departure: just use the latest drone return
        truck_departure[i] = std::max(truck_arrival[i], latest_drone_return_so_far);

        if (curr != 0) // depot
            total += truck_arrival[i]; // truck waiting
    }

    return total / 100; // scale units
}

struct DroneFlight {
    int launch_idx;
    int customer;
    int land_idx;
};

/**
 */
long long objective_function_impl(const Instance& instance, const Solution& solution) {
    std::vector<long long> drone_available;
    long long total_drone_arrival = 0;

    // Compute truck arrival times and accumulate drone arrival times
    std::vector<long long> t_arrival = get_truck_arrival_times_at_node(
        instance, solution, drone_available, total_drone_arrival
    );

    long long total_time = total_drone_arrival;

    // Add truck arrival times (excluding depot)
    for (int i = 1; i < static_cast<int>(solution.truck_route.size()); ++i) {
        total_time += t_arrival[i];
    }

    // Scale units (same as Python's division by 100.0, but integer truncation here)
    return total_time / 100;
}