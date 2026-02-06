#include "algorithms/random_valid_solution.h"
#include <algorithm>
#include <vector>
#include <random>
#include <iostream>
#include <numeric>

const int SEED = 42;
std::mt19937 gen(SEED);

int randInt(int a, int b) {
    std::uniform_int_distribution<> dist(a, b);
    return dist(gen);
}

template <typename T>
std::vector<T> random_sample(const std::vector<T>& vec, size_t k) {
    std::vector<T> copy = vec;
    std::shuffle(copy.begin(), copy.end(), gen);
    copy.resize(k);
    return copy;
}

Solution random_valid_solution(int n) {
    Solution sol;

    // ----- Truck route -----
    std::vector<int> nodes(n);
    std::iota(nodes.begin(), nodes.end(), 1);

    int k_truck = randInt(n / 3, n);
    std::vector<int> truck_nodes = random_sample(nodes, k_truck);

    sol.truck_route.push_back(0);
    sol.truck_route.insert(sol.truck_route.end(), truck_nodes.begin(), truck_nodes.end());
    sol.truck_route.push_back(0);

    // ----- Drone delivery nodes -----
    std::vector<int> remaining;
    for (int node : nodes) {
        if (std::find(truck_nodes.begin(), truck_nodes.end(), node) == truck_nodes.end())
            remaining.push_back(node);
    }
    std::shuffle(remaining.begin(), remaining.end(), gen);

    sol.drones.resize(2);

    int drone_counter = 0;
    while (!remaining.empty()) {
        int delivery_node = remaining.back();
        remaining.pop_back();

        // Choose launch and land nodes from truck (excluding depot)
        std::vector<int> truck_mid(sol.truck_route.begin() + 1, sol.truck_route.end() - 1);
        std::shuffle(truck_mid.begin(), truck_mid.end(), gen);

        int launch_node = truck_mid[0];
        int land_node = truck_mid[1];
        if (launch_node == land_node) land_node = truck_mid[1 % truck_mid.size()];

        int launch_idx = std::find(sol.truck_route.begin(), sol.truck_route.end(), launch_node) - sol.truck_route.begin();
        int land_idx   = std::find(sol.truck_route.begin(), sol.truck_route.end(), land_node) - sol.truck_route.begin();
        if (launch_idx > land_idx) std::swap(launch_idx, land_idx), std::swap(launch_node, land_node);

        DroneTrip trip;
        trip.delivery_node = delivery_node;
        trip.launch_node = launch_node;
        trip.land_node = land_node;
        trip.launch_index = launch_idx;
        trip.land_index = land_idx;

        // Assign trip to one of the drones randomly
        int d = drone_counter % 2;
        sol.drones[d].push_back(trip);
        drone_counter++;
    }

    return sol;
}
