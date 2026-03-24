#include "verification/feasibility_check.h"
#include "operators/route_timing.h"
#include <algorithm>
#include <iostream>

static bool truck_route_feasible(const Instance& instance, const Solution& solution, bool debug) {
    if (solution.truck_route.empty()) {
        if (debug) {
            std::cout << "Truck route is empty.\n";
        }
        return false;
    }

    if (solution.truck_route[0] != 0) {
        if (debug) {
            std::cout << "Truck route must start at depot (0).\n";
        }
        return false;
    }

    for (int i = 0; i < (int)solution.truck_route.size(); ++i) {
        int node = solution.truck_route[i];
        if (node < 0 || node > instance.n) {
            if (debug) {
                std::cout << "Invalid node in truck route: " << node << "\n";
            }
            return false;
        }
        if (i > 0 && node == 0) {
            if (debug) {
                std::cout << "Depot appears in the middle of the truck route.\n";
            }
            return false;
        }
    }

    return true;
}

bool includes_all_nodes(int n, const Solution& solution, bool debug) {
    // Nodes are 0..n, with 0 as depot and customers 1..n.
    std::vector<int> count(n + 1, 0);
    bool ok = true;

    for (int i = 0; i < (int)solution.truck_route.size(); ++i) {
        int node = solution.truck_route[i];
        if (node < 0 || node > n) {
            if (debug) {
                std::cout << "Invalid node in truck route: " << node << "\n";
            }
            ok = false;
            continue;
        }
        if (node == 0) {
            // depot is handled by truck_route_feasible
            continue;
        }
        count[node]++;
    }

    for (const DroneCollection& c : solution.drones) {
        if (c.launch_indices.size() != c.land_indices.size() ||
            c.launch_indices.size() != c.deliver_nodes.size()) {
            if (debug) {
                std::cout << "Drone collection sizes are inconsistent.\n";
            }
            ok = false;
            continue;
        }

        for (int node : c.deliver_nodes) {
            if (node < 0 || node > n) {
                if (debug) {
                    std::cout << "Invalid node in drone delivery: " << node << "\n";
                }
                ok = false;
                continue;
            }
            if (node == 0) {
                if (debug) {
                    std::cout << "Depot cannot be delivered by drone.\n";
                }
                ok = false;
                continue;
            }
            count[node]++;
        }
    }

    for (int i = 1; i <= n; i++) {
        if (count[i] != 1) {
            if (debug) {
                std::cout << "Customer " << i << " covered " << count[i] << " times.\n";
            }
            ok = false;
        }
    }

    return ok;
}

bool specific_drone_flight_under_lim(const Instance& instance,
                                     const Solution& solution,
                                     int drone_idx, int flight_idx) {
    const DroneCollection& c = solution.drones[drone_idx];
    int launch_idx = c.launch_indices[flight_idx];
    int land_idx   = c.land_indices[flight_idx];
    int deliver    = c.deliver_nodes[flight_idx];

    if (launch_idx < 0 || land_idx < 0 || deliver < 0) return false;

    // Check indices are within the truck route
    if (launch_idx >= (int)solution.truck_route.size() || land_idx >= (int)solution.truck_route.size()) {
        return false;
    }

    int launch_node = solution.truck_route[launch_idx];
    int land_node = solution.truck_route[land_idx];

    long long drone_total_duration =
        instance.drone_matrix[launch_node][deliver] + instance.drone_matrix[deliver][land_node];

    return drone_total_duration <= instance.lim;
}

bool all_drone_flights_under_lim_with_wait(const Instance& instance,
                                           const Solution& solution,
                                           bool debug = false) {
    const int route_size = (int)solution.truck_route.size();
    if (route_size == 0) {
        return false;
    }

    for (int d = 0; d < (int)solution.drones.size(); ++d) {
        const DroneCollection& c = solution.drones[d];
        if (c.launch_indices.size() != c.land_indices.size() ||
            c.launch_indices.size() != c.deliver_nodes.size()) {
            if (debug) {
                std::cout << "Drone collection sizes are inconsistent for drone " << d << ".\n";
            }
            return false;
        }
    }

    const RouteTiming timing = compute_route_timing(instance, solution);
    bool all_ok = true;

    for (int d = 0; d < (int)solution.drones.size(); ++d) {
        const DroneCollection& c = solution.drones[d];
        for (int t = 0; t < (int)c.launch_indices.size(); ++t) {
            const int launch_idx = c.launch_indices[t];
            const int land_idx = c.land_indices[t];
            const int deliver = c.deliver_nodes[t];

            if (launch_idx < 0 || land_idx < 0 ||
                launch_idx >= route_size || land_idx >= route_size ||
                deliver <= 0 || deliver > instance.n) {
                all_ok = false;
                if (debug) {
                    std::cout << "Invalid drone flight data: launch=" << launch_idx
                              << " land=" << land_idx << " deliver=" << deliver << "\n";
                }
                continue;
            }

            const int launch_node = solution.truck_route[launch_idx];
            const int land_node = solution.truck_route[land_idx];
            const long long launch_time = std::max(
                timing.truck_arrival[launch_idx],
                timing.drone_ready_at_stop[d][launch_idx]);
            const long long out_time = instance.drone_matrix[launch_node][deliver];
            const long long back_time = instance.drone_matrix[deliver][land_node];
            const long long drone_return = launch_time + out_time + back_time;
            const long long drone_wait = land_node == 0
                ? 0
                : std::max(timing.truck_arrival[land_idx] - drone_return, 0LL);
            const long long total_with_wait = out_time + back_time + drone_wait;

            if (total_with_wait > instance.lim) {
                all_ok = false;
                if (debug) {
                    std::cout << "Drone flight exceeds limit with wait: "
                              << launch_node << " -> " << deliver << " -> " << land_node
                              << ", duration: " << total_with_wait
                              << ", limit: " << instance.lim << "\n";
                }
            }
        }
    }

    return all_ok;
}
bool drone_flights_consistent(const Solution& solution, bool debug = false) {
    int route_size = (int)solution.truck_route.size();

    for (const DroneCollection& c : solution.drones) {
        if (c.launch_indices.size() != c.land_indices.size() ||
            c.launch_indices.size() != c.deliver_nodes.size()) {
            if (debug) {
                std::cout << "Drone collection sizes are inconsistent.\n";
            }
            return false;
        }

        std::vector<std::pair<int, int>> flights;
        flights.reserve(c.launch_indices.size());

        for (int i = 0; i < (int)(c.launch_indices.size()); i++) {
            int launch_idx = c.launch_indices[i];
            int land_idx = c.land_indices[i];

            if (launch_idx < 0 || land_idx < 0 ||
                launch_idx >= route_size || land_idx >= route_size) {
                if (debug) {
                    std::cout << "Drone trip out of range: launch_index = "
                              << launch_idx << ", land_index = "
                              << land_idx << "\n";
                }
                return false;
            }

            if (land_idx <= launch_idx) {
                if (debug) {
                    std::cout << "Drone trip inconsistent: launch_index = " 
                              << launch_idx << ", land_index = "
                              << land_idx << "\n";
                }
                return false;
            }

            flights.push_back({launch_idx, land_idx});
        }

        std::sort(flights.begin(), flights.end(),
                  [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                      return a.first < b.first;
                  });

        for (int i = 1; i < (int)flights.size(); ++i) {
            if (flights[i].first < flights[i - 1].second) {
                if (debug) {
                    std::cout << "Drone trips overlap: prev land=" << flights[i - 1].second
                              << ", next launch=" << flights[i].first << "\n";
                }
                return false;
            }
        }
    }
    return true;
}

bool all_drone_flights_feasible(const Instance& problem_instance, const Solution& solution, bool debug) {
    return (all_drone_flights_under_lim_with_wait(problem_instance, solution, debug) &&
            drone_flights_consistent(solution, debug));
}

bool master_check(const Instance& problem_instance, const Solution& solution, bool debug) {
    return (truck_route_feasible(problem_instance, solution, debug) &&
            includes_all_nodes(problem_instance.n, solution, debug) &&
            all_drone_flights_feasible(problem_instance, solution, debug));
}




