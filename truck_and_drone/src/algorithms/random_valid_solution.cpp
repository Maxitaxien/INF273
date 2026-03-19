#include "algorithms/random_valid_solution.h"
#include "general/random.h"
#include "general/get_not_covered_by_truck.h"
#include <algorithm>
#include <vector>
#include <random>
#include <iostream>
#include <numeric>



Solution random_valid_solution(int n) {
    Solution sol;

    // ----- Truck route -----
    std::vector<int> nodes(n);
    std::iota(nodes.begin(), nodes.end(), 1);

    int k_truck = rand_int((n / 2) + 1, n);
    std::vector<int> truck_nodes = random_sample(nodes, k_truck);

    sol.truck_route.push_back(0);
    sol.truck_route.insert(sol.truck_route.end(), truck_nodes.begin(), truck_nodes.end());

    // ----- Drone delivery nodes -----
    std::vector<int> remaining = get_not_covered_by_truck(n, sol);
    random_shuffle(remaining);

    sol.drones.resize(2);

    // ----- Determine amount assigned to each drone -----
    int R = remaining.size();
    int T = sol.truck_route.size(); // includes depot now

    // Max deliveries a drone can have without violating uniqueness
    int max_per_drone = T / 2;

    // Feasible min/max assignments
    int min_assign = std::max(0, R - max_per_drone);
    int max_assign = std::min(R, max_per_drone);

    // Clamp if somehow infeasible
    if (min_assign > max_assign) min_assign = max_assign = std::min(R, max_per_drone);

    int k_assign = rand_int(min_assign, max_assign);

    // Split remaining nodes between the two drones
    std::pair<std::vector<int>, std::vector<int>> drone_assignments = random_partition(remaining, k_assign);
    auto& drone1_nodes = drone_assignments.first;
    auto& drone2_nodes = drone_assignments.second;

    // truck route indices for launch/land (0-based, depot included)
    std::vector<int> truck_indices(T);
    std::iota(truck_indices.begin(), truck_indices.end(), 0);

    // Lambda to assign launches, deliveries, and landings
    auto assign_drone = [&](int d, const std::vector<int>& deliveries) {
        int m = deliveries.size();
        int T = sol.truck_route.size(); // total truck route indices including depot

        int last_land = 0; // start at depot

        for (int i = 0; i < m; i++) {
            // remaining deliveries including this one
            int remaining_deliveries = m - i;

            // max launch index = T - remaining_deliveries*2
            int max_launch = T - remaining_deliveries * 2;
            if (last_land > max_launch) last_land = max_launch;

            int launch_idx = rand_int(last_land, max_launch);
            int land_idx   = rand_int(launch_idx + 1, T - remaining_deliveries);

            last_land = land_idx; // update last landing

            sol.drones[d].launch_indices.push_back(launch_idx);
            sol.drones[d].land_indices.push_back(land_idx);
            sol.drones[d].deliver_nodes.push_back(deliveries[i]);
        }
    };



    // Assign both drones
    assign_drone(0, drone1_nodes);
    assign_drone(1, drone2_nodes);


    return sol;
}
