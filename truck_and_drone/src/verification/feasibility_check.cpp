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
        if (node != 0) {
            is_covered[node] = !is_covered[node];
        }
        else {
            is_covered[node] = true;
        }
    }

    for (DroneCollection c : solution.drones) {
        for (int node : c.deliver_nodes) {
            if (node != 0) {
                is_covered[node] = true;
            }
            else {
                is_covered[node] = true;
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

bool all_drone_flights_under_lim(const Instance& problem_instance, const Solution& solution, bool debug = false) {
    bool all_ok = true;

    for (DroneCollection c : solution.drones) {
        for (int i = 0; i < c.launch_indices.size(); i++) {
            int deliver = c.deliver_nodes[i];
            int launch = solution.truck_route[c.launch_indices[i]];
            int land = solution.truck_route[c.land_indices[i]];
            long long drone_time = problem_instance.drone_matrix[launch][deliver] +
                                   problem_instance.drone_matrix[deliver][land];

            long long truck_arrival = problem_instance.truck_matrix[launch][land];

            long long effective_drone_time = std::max(drone_time, truck_arrival);

            if (effective_drone_time > problem_instance.lim) {
                if (debug) {
                    std::cout << "Illegal drone trip: "
                              << launch << " -> " << deliver << " -> " << land << "\n"
                              << "Lim: " << problem_instance.lim
                              << ", effective drone time: " << effective_drone_time
                              << " (drone: " << drone_time
                              << ", truck: " << truck_arrival << ")\n";
                }
                all_ok = false;
            }
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