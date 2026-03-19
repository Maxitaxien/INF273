#include "general/get_not_covered_by_truck.h"
#include <vector>

std::vector<int> get_not_covered_by_truck(int n, const Solution &sol) {
    std::vector<bool> seen(n + 1, false);

    for (int c : sol.truck_route) {
        seen[c] = true;
    }

    std::vector<int> missing;
    for (int i = 0; i <= n; i++) {
        if (!seen[i]) {
            missing.push_back(i);
        }
    }

    return missing;
}

std::vector<int> get_not_covered_by_truck(const Solution &sol) {
    size_t total = 0;
    for (const DroneCollection &drone : sol.drones) {
        total += drone.deliver_nodes.size();
    }

    std::vector<int> covered_by_drone;
    covered_by_drone.reserve(total);
    for (const DroneCollection &drone : sol.drones) {
        covered_by_drone.insert(
            covered_by_drone.end(),
            drone.deliver_nodes.begin(),
            drone.deliver_nodes.end());
    }

    return covered_by_drone;
}
