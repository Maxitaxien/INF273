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

bool includes_all_nodes(int n, const std::string& submission) {
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
    else {
        for (int i = 0; i < is_covered.size(); i++) {
            if (!is_covered[i]) {
                std::cout << is_covered[i] << " not covered or covered multiple times.\n";
            }
        }
    }
    return false;
}

bool all_drone_flights_under_lim(const Instance& problem_instance, const Solution& solution) {
    bool all_ok = true;

    for (const auto& [launch_node, drone_flights] : solution.drone_map) {
        for (const auto& tup : drone_flights) {
            int target = std::get<0>(tup);
            int landing = std::get<1>(tup);

            // Only consider assigned flights
            if (target != -1 && landing != -1) {
                long long drone_time = problem_instance.drone_matrix[launch_node][target] +
                                       problem_instance.drone_matrix[target][landing];

                long long truck_arrival_at_landing = problem_instance.truck_matrix[launch_node][landing];

                // Effective drone time includes waiting for truck if it arrives late
                long long effective_drone_time = std::max(drone_time, truck_arrival_at_landing);

                if (effective_drone_time > problem_instance.lim) {
                    std::cout << "Found illegal drone tour: "
                              << launch_node << " -> " << target << " -> " << landing << "\n";
                    std::cout << "Lim: " << problem_instance.lim
                              << ", effective drone time: " << effective_drone_time
                              << " (drone time: " << drone_time
                              << ", truck arrival: " << truck_arrival_at_landing << ")\n\n";
                    all_ok = false;
                }
            }
        }
    }

    return all_ok;
}

bool drone_flights_consistent(const Solution& solution) {
    // Build index lookup for all nodes
    std::unordered_map<int, int> node_position;
    for (int i = 0; i < solution.truck_route.size() - 1; i++) {
        node_position[solution.truck_route[i]] = i;
    }

    bool all_ok = true;
    for (const auto& [launch, vec] : solution.drone_map) {
        for (const auto& tup : vec) {
            int deliver = std::get<0>(tup);
            int land = std::get<1>(tup);
            
            if (deliver == -1 && land == -1) continue;
            
            int truck_start = node_position[launch];
            int truck_end = node_position[land];
            if (truck_end <= truck_start) {
                std::cout << "Found illegal drone launch: "
                          << launch << " -> " << deliver << " -> " << land << "\n"; 
                std::cout << "Launch node " << launch << " visited at time: " << truck_start << "\n";
                std::cout << "Land node " << land  << " visited at time: " << truck_end << "\n";
                all_ok = false;
            }
        }
    }
    return all_ok;
}