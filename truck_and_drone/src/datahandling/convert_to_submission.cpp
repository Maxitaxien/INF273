#include "datahandling/convert_to_submission.h"
#include <vector>
#include <algorithm>
#include <tuple>
#include <string>

std::string convert_to_submission(const Solution& solution) {
    std::string submission;

    // ---- Truck route ----
    for (int n : solution.truck_route) {
        submission += std::to_string(n) + ",";
    }
    if (!submission.empty())
        submission.pop_back(); // remove last comma
    submission += "|";

    constexpr int DRONES = 2;

    // Prepare per-drone vectors
    std::vector<int> deliver_nodes[DRONES];
    std::vector<int> launch_indices[DRONES];
    std::vector<int> land_indices[DRONES];

    // Sort each drone’s trips by launch index
    for (int d = 0; d < DRONES; d++) {
        if (d >= solution.drones.size()) continue;

        for (auto& trip : solution.drones[d]) {
            deliver_nodes[d].push_back(trip.delivery_node);
            launch_indices[d].push_back(trip.launch_index + 1);
            land_indices[d].push_back(trip.land_index + 1);
        }
    }

    // Sort launch and land indices numerically
    for (int d = 0; d < DRONES; d++) {
        std::sort(launch_indices[d].begin(), launch_indices[d].end());
        std::sort(land_indices[d].begin(), land_indices[d].end());
    }

    
    // ---- Deliveries ----
    auto append_deliveries = [&](int d){
        for (int node : deliver_nodes[d]) submission += std::to_string(node) + ",";
    };

    append_deliveries(0);
    submission += "-1,"; // separator
    append_deliveries(1);
    if (!submission.empty())
        submission.pop_back();
    submission += "|";

    // ---- Launch ----
    for (int x : launch_indices[0]) submission += std::to_string(x) + ",";
    submission += "-1,";
    for (int x : launch_indices[1]) submission += std::to_string(x) + ",";
    if (!submission.empty())
        submission.pop_back();
    submission += "|";

    // ---- Land ----
    for (int x : land_indices[0]) submission += std::to_string(x) + ",";
    submission += "-1,";
    for (int x : land_indices[1]) submission += std::to_string(x) + ",";
    if (!submission.empty())
        submission.pop_back();

    return submission;
}
