#include "verification/feasibility_check.h"
#include <sstream>
#include <algorithm>
#include <iostream>

std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

bool includes_all_nodes(int n, const Solution& solution, bool debug) {
    std::vector<bool> is_covered(n + 1, false);

    for (int node : solution.truck_route) {
        if (!is_covered[node]) {
            is_covered[node] = true;
        }
        else {
            std::cout << is_covered[node] << " not covered or covered multiple times.\n";
        }
    }

    for (const DroneCollection& c : solution.drones) {
        for (int node : c.deliver_nodes) {
            if (!is_covered[node]) {
                is_covered[node] = true;
            }
            else {
                std::cout << is_covered[node] << " not covered or covered multiple times.\n";
            }
        }
    }

    if (std::all_of(is_covered.begin(), is_covered.end(), [](bool b) { return b; })) {
        return true;
    }
    if (debug) {
        for (int i = 0; i < is_covered.size(); i++) {
            if (!is_covered[i]) {
                std::cout << is_covered[i] << " not covered or covered multiple times.\n";
            }
        }
    }
    return false;
}

bool includes_all_nodes(int n, const std::string& submission, bool debug) {
    std::vector<bool> is_covered(n + 1, false);

    auto parts = split(submission, '|');

    auto covered = std::vector<std::string>(parts.begin(), parts.begin() + 2);

    for (const std::string& s : covered) {
        for (const std::string& c : split(s, ',')) {
            if (c != "-1") {
                int node = std::stoi(c);
                if (c != "0") {
                    is_covered[node] = !is_covered[node];
                }
                else {
                    is_covered[node] = true;
                }
            }
        }
    }

    if (std::all_of(is_covered.begin(), is_covered.end(), [](bool b) { return b; })) {
        return true;
    }
    if (debug) {
        for (int i = 0; i < is_covered.size(); i++) {
            if (!is_covered[i]) {
                std::cout << is_covered[i] << " not covered or covered multiple times.\n";
            }
        }
    }
    return false;
}

bool specific_drone_flight_under_lim(const Instance& instance,
                                     const Solution& solution,
                                     int drone_idx, int flight_idx) {
    const DroneCollection& c = solution.drones[drone_idx];
    int launch_idx = c.launch_indices[flight_idx];
    int land_idx   = c.land_indices[flight_idx];
    int deliver    = c.deliver_nodes[flight_idx];

    if (launch_idx == -1 || land_idx == -1 || deliver == -1) return false;

    std::vector<long long> drone_available;
    std::vector<long long> truck_arrival = get_truck_arrival_times_at_node(instance, solution, drone_available);

    long long launch_time_allowed = std::max(truck_arrival[launch_idx], drone_available[drone_idx]);

    long long drone_total_duration = (launch_time_allowed - truck_arrival[launch_idx]) // waiting for truck
                                    + instance.drone_matrix[solution.truck_route[launch_idx]][deliver] // out
                                    + instance.drone_matrix[deliver][solution.truck_route[land_idx]]; // back

    return drone_total_duration <= instance.lim;
}

// Returns truck arrival times at each node and updates drone availability dynamically
std::vector<long long> get_truck_arrival_times_at_node(
    const Instance& instance,
    const Solution& solution,
    std::vector<long long>& drone_available // output parameter
) {
    int m = solution.truck_route.size();
    std::vector<long long> truck_arrival(m, 0);
    std::vector<long long> truck_departure(m, 0);

    drone_available.assign(solution.drones.size(), 0);
    long long latest_drone_return_so_far = 0;

    // Precompute which drones land at which truck index
    std::vector<std::vector<std::pair<int,int>>> drone_returns_at(m);
    for (int d = 0; d < solution.drones.size(); ++d) {
        const DroneCollection& c = solution.drones[d];
        for (int t = 0; t < c.launch_indices.size(); ++t) {
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

        // Update drones landing at this stop
        for (auto [d, t] : drone_returns_at[i]) {
            const DroneCollection& c = solution.drones[d];

            int launch_idx   = c.launch_indices[t];
            int launch_node  = solution.truck_route[launch_idx];
            int deliver_node = c.deliver_nodes[t];
            int land_node    = curr;

            long long out_time  = instance.drone_matrix[launch_node][deliver_node];
            long long back_time = instance.drone_matrix[deliver_node][land_node];

            long long launch_time   = std::max(truck_arrival[launch_idx], drone_available[d]);
            long long drone_return  = launch_time + out_time + back_time;

            drone_available[d] = drone_return;
            latest_drone_return_so_far = std::max(latest_drone_return_so_far, drone_return);
        }

        truck_departure[i] = std::max(truck_arrival[i], latest_drone_return_so_far);
    }

    return truck_arrival;
}


bool all_drone_flights_under_lim(const Instance& instance,
                                 const Solution& solution,
                                 bool debug = false) {
    std::vector<long long> drone_available;
    std::vector<long long> truck_arrival = get_truck_arrival_times_at_node(instance, solution, drone_available);

    bool all_ok = true;

    for (int d = 0; d < solution.drones.size(); ++d) {
        const DroneCollection& c = solution.drones[d];
        for (int i = 0; i < c.launch_indices.size(); ++i) {
            int launch_idx = c.launch_indices[i];
            int land_idx   = c.land_indices[i];
            int deliver    = c.deliver_nodes[i];

            if (launch_idx == -1 || land_idx == -1 || deliver == -1) continue;

           long long launch_time_allowed = std::max(truck_arrival[launch_idx], drone_available[d]);

            long long drone_total_duration = (launch_time_allowed - truck_arrival[launch_idx])
                                            + instance.drone_matrix[solution.truck_route[launch_idx]][deliver]
                                            + instance.drone_matrix[deliver][solution.truck_route[land_idx]];

            if (drone_total_duration > instance.lim) {
                all_ok = false;
                if (debug) {
                    std::cout << "Drone flight exceeds limit including wait: "
                            << solution.truck_route[launch_idx] << " -> " << deliver << " -> "
                            << solution.truck_route[land_idx]
                            << ", duration: " << drone_total_duration
                            << ", limit: " << instance.lim << "\n";
                }
            }
            drone_available[d] = launch_time_allowed + instance.drone_matrix[solution.truck_route[launch_idx]][deliver]
                                                + instance.drone_matrix[deliver][solution.truck_route[land_idx]];
                    }
    }

    return all_ok;
}

bool drone_flights_consistent(const Solution& solution, bool debug = false) {
    for (DroneCollection c : solution.drones) {
        for (int i = 0; i < c.launch_indices.size(); i++) {
            int launch_idx = c.launch_indices[i];
            int land_idx = c.land_indices[i];
            if (land_idx <= launch_idx) {
                if (debug) {
                    std::cout << "Drone trip inconsistent: launch_index = " 
                              << launch_idx << ", land_index = "
                              << land_idx << "\n";
                }
                return false;
            }
        }
    }
    return true;
}

bool all_drone_flights_feasible(const Instance& problem_instance, const Solution& solution, bool debug) {
    return (all_drone_flights_under_lim(problem_instance, solution, debug) && drone_flights_consistent(solution, debug));
}

bool master_check(const Instance& problem_instance, const Solution& solution, bool debug) {
    return (all_drone_flights_feasible(problem_instance, solution, debug) && includes_all_nodes(problem_instance.n, solution, debug));
}