#include "verification/feasibility_check.h"
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
    const int m = static_cast<int>(solution.truck_route.size());
    if (m == 0) return false;

    // Precompute drone returns by truck index
    std::vector<std::vector<std::pair<int, int>>> drone_returns_at(m);
    for (int d = 0; d < static_cast<int>(solution.drones.size()); ++d) {
        const DroneCollection& c = solution.drones[d];
        if (c.launch_indices.size() != c.land_indices.size() ||
            c.launch_indices.size() != c.deliver_nodes.size()) {
            if (debug) {
                std::cout << "Drone collection sizes are inconsistent for drone " << d << ".\n";
            }
            return false;
        }

        for (int t = 0; t < static_cast<int>(c.launch_indices.size()); ++t) {
            int land_idx = c.land_indices[t];
            if (land_idx < 0 || land_idx >= m) {
                if (debug) {
                    std::cout << "Invalid land index: " << land_idx << " for drone " << d << "\n";
                }
                return false;
            }
            drone_returns_at[land_idx].emplace_back(d, t);
        }
    }

    std::vector<long long> truck_arrival(m, 0);
    std::vector<long long> truck_departure(m, 0);
    std::vector<long long> drone_available(solution.drones.size(), 0);

    long long latest_drone_return_so_far = 0;
    bool all_ok = true;

    for (int i = 1; i < m; ++i) {
        int prev = solution.truck_route[i - 1];
        int curr = solution.truck_route[i];

        truck_arrival[i] = truck_departure[i - 1] + instance.truck_matrix[prev][curr];

        // Process drones landing at this stop
        for (const auto& entry : drone_returns_at[i]) {
            int d = entry.first;
            int t = entry.second;
            const DroneCollection& c = solution.drones[d];

            int launch_idx = c.launch_indices[t];
            int land_idx = c.land_indices[t];
            int deliver = c.deliver_nodes[t];

            if (launch_idx < 0 || launch_idx >= m || deliver < 0 || deliver > instance.n) {
                all_ok = false;
                if (debug) {
                    std::cout << "Invalid drone flight data: launch=" << launch_idx
                              << " land=" << land_idx << " deliver=" << deliver << "\n";
                }
                continue;
            }

            int launch_node = solution.truck_route[launch_idx];
            int land_node = solution.truck_route[land_idx];

            long long out_time = instance.drone_matrix[launch_node][deliver];
            long long back_time = instance.drone_matrix[deliver][land_node];
            long long total_flight = out_time + back_time;

            long long launch_time = std::max(truck_arrival[launch_idx], drone_available[d]);
            long long drone_return = launch_time + total_flight;

            long long drone_wait = 0;
            if (curr != 0) {
                drone_wait = std::max(truck_arrival[i] - drone_return, 0LL);
            }

            long long total_with_wait = total_flight + drone_wait;
            if (total_with_wait > instance.lim) {
                all_ok = false;
                if (debug) {
                    std::cout << "Drone flight exceeds limit with wait: "
                              << launch_node << " -> " << deliver << " -> " << land_node
                              << ", duration: " << total_with_wait
                              << ", limit: " << instance.lim << "\n";
                }
            }

            drone_available[d] = drone_return;
            latest_drone_return_so_far = std::max(latest_drone_return_so_far, drone_return);
        }

        truck_departure[i] = std::max(truck_arrival[i], latest_drone_return_so_far);
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

        for (int i = 0; i < c.launch_indices.size(); i++) {
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
