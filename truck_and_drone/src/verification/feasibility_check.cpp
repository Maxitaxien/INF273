#include "verification/feasibility_check.h"
#include "general/split.h"
#include "general/get_truck_arrival_times.h"
#include <algorithm>
#include <iostream>

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
    long long total_drone_arrival;
    std::vector<long long> truck_arrival = get_truck_arrival_times(instance, solution, drone_available, total_drone_arrival);

    long long launch_time_allowed = std::max(truck_arrival[launch_idx], drone_available[drone_idx]);

    long long drone_total_duration = (launch_time_allowed - truck_arrival[launch_idx]) // waiting for truck
                                    + instance.drone_matrix[solution.truck_route[launch_idx]][deliver] // out
                                    + instance.drone_matrix[deliver][solution.truck_route[land_idx]]; // back

    return drone_total_duration <= instance.lim;
}


bool all_drone_flights_under_lim(const Instance& instance,
                                 const Solution& solution,
                                 bool debug = false) {
    std::vector<long long> drone_available;
    long long total_drone_arrival;
    std::vector<long long> truck_arrival = get_truck_arrival_times(instance, solution, drone_available, total_drone_arrival);

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