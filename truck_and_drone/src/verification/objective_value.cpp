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
    long long total_time = 0;
    int D = solution.drones.size();

    std::vector<long long> drone_available(D, 0);
    std::vector<long long> t_arrival = get_truck_arrival_times_at_node(instance, solution, drone_available);

    for (int i = 1; i < solution.truck_route.size(); ++i) {
        int curr_node = solution.truck_route[i];

        long long latest_drone_return = 0;

        for (int d = 0; d < D; ++d) {
            const DroneCollection& c = solution.drones[d];
            for (size_t f = 0; f < c.deliver_nodes.size(); ++f) {
                if (c.land_indices[f] == i) {
                    int launch_node = solution.truck_route[c.launch_indices[f]];
                    int cust_node   = c.deliver_nodes[f];

                    long long out_time  = instance.drone_matrix[launch_node][cust_node];
                    long long back_time = instance.drone_matrix[cust_node][curr_node];

                    long long actual_launch = std::max(t_arrival[c.launch_indices[f]], drone_available[d]);
                    long long drone_arrival = actual_launch + out_time;
                    long long drone_return  = drone_arrival + back_time;

                    total_time += drone_arrival;      // customer wait contribution
                    latest_drone_return = std::max(latest_drone_return, drone_return);

                    drone_available[d] = drone_return;
                }
            }
        }

        // Truck contribution: only arrival at node (same as Python)
        total_time += t_arrival[i];

        // Do NOT add truck_wait to total_time
        // truck_wait only affects departure for next leg
    }

    return total_time / 100; // scale units
}

long long compute_total_wait_time(const Instance& instance, const Solution& solution) {
    int n = instance.n;
    int m = instance.m;

    const std::vector<int>& truck_route = solution.truck_route;

    // 1. Track truck arrival and departure times at each node in the route
    std::vector<long long> truck_arrival(truck_route.size(), 0);
    std::vector<long long> truck_departure(truck_route.size(), 0);

    // 2. Drone availability times
    std::vector<long long> drone_available(m, 0);

    // 3. Total time accumulator (like Python total_time)
    long long total_time = 0;

    // 4. Drone deliveries
    for (int d = 0; d < m; ++d) {
        const DroneCollection& drone = solution.drones[d];
        for (size_t t = 0; t < drone.launch_indices.size(); ++t) {
            int launch_idx = drone.launch_indices[t];   // index in truck_route
            int land_idx = drone.land_indices[t];       // index in truck_route
            int delivery_node = drone.deliver_nodes[t]; // customer node

            int launch_node = truck_route[launch_idx];
            int land_node = truck_route[land_idx];

            // Drone can start only when truck is at launch node and drone is available
            long long start_time = std::max(drone_available[d], truck_arrival[launch_idx]);

            // Flight time: launch -> delivery -> land
            long long flight_time = instance.drone_matrix[launch_node][delivery_node]
                                  + instance.drone_matrix[delivery_node][land_node];

            // Customer arrival
            long long drone_arrival_customer = start_time + instance.drone_matrix[launch_node][delivery_node];

            // Update drone availability
            drone_available[d] = start_time + flight_time;

            // Add to total_time like Python
            total_time += drone_arrival_customer;
        }
    }

    // 5. Compute truck arrival and departure times iteratively, accounting for drone returns
    for (size_t i = 1; i < truck_route.size(); ++i) { // skip depot
        int prev = truck_route[i - 1];
        int curr = truck_route[i];

        // Truck travel from previous node
        long long travel_time = instance.truck_matrix[prev][curr];
        truck_arrival[i] = truck_departure[i - 1] + travel_time;

        // Check if any drones return to this node
        std::vector<long long> returning_drones;
        for (int d = 0; d < m; ++d) {
            const DroneCollection& drone = solution.drones[d];
            for (size_t t = 0; t < drone.land_indices.size(); ++t) {
                if (drone.land_indices[t] == i) {
                    returning_drones.push_back(drone_available[d]);
                }
            }
        }

        // Truck waits for latest returning drone
        if (!returning_drones.empty()) {
            truck_departure[i] = std::max(truck_arrival[i], *std::max_element(returning_drones.begin(), returning_drones.end()));
        } else {
            truck_departure[i] = truck_arrival[i];
        }

        // If this node was delivered by truck only (no drone), add its arrival to total_time
        bool delivered_by_drone = false;
        for (int d = 0; d < m; ++d) {
            const DroneCollection& drone = solution.drones[d];
            for (size_t t = 0; t < drone.deliver_nodes.size(); ++t) {
                if (drone.deliver_nodes[t] == curr) {
                    delivered_by_drone = true;
                    break;
                }
            }
            if (delivered_by_drone) break;
        }
        if (!delivered_by_drone && curr != 0) {
            total_time += truck_arrival[i];
        }
    }

    return total_time / 100;
}