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

    for (int i : solution.truck_route) {
        if (i != 0) {
            is_covered[i] = !is_covered[i];
        }
        else {
            is_covered[i] = true;
        }
    }

    for (const auto& drone_list : solution.drones) {
        for (const auto& trip : drone_list) {
            if (trip.delivery_node != -1) {
                is_covered[trip.delivery_node] = true;
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

    for (const auto& drone_list : solution.drones) {
        for (const auto& trip : drone_list) {
            int launch = trip.launch_node;
            int deliver = trip.delivery_node;
            int land = trip.land_node;

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
    for (const auto& drone_list : solution.drones) {
        for (const auto& trip : drone_list) {
            if (trip.launch_index == -1 || trip.land_index == -1) continue;
            if (trip.land_index <= trip.launch_index) {
                if (debug) {
                    std::cout << "Drone trip inconsistent: launch_index = " 
                              << trip.launch_index << ", land_index = "
                              << trip.land_index << "\n";
                }
                return false;
            }
            if (trip.launch_node == trip.land_node) {
                if (debug) {
                    std::cout << "Drone trip invalid: launch_node == land_node (" 
                              << trip.launch_node << ")\n";
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