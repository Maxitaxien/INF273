#include "datahandling/convert_to_submission.h"
#include "general/sort_drone_collection.h"
#include <vector>
#include <tuple>
#include <string>

std::string convert_to_submission(const Solution& solution) {
    std::string submission;

    // ---- Truck route ----
    for (int n : solution.truck_route) {
        submission += std::to_string(n) + ",";
    }
    submission.push_back('0');
    submission += "|";

    constexpr int DRONES = 2;

    // Prepare per-drone collections, sorted by launch index
    DroneCollection sorted[DRONES];
    for (int d = 0; d < DRONES; d++) {
        if (d >= solution.drones.size()) continue;
        sorted[d] = solution.drones[d];
        sort_drone_collection(sorted[d]);
    }

    // ---- Deliveries ----
    auto append_deliveries = [&](int d){
        for (int node : sorted[d].deliver_nodes) submission += std::to_string(node) + ",";
    };

    append_deliveries(0);
    submission += "-1,"; // separator
    append_deliveries(1);
    if (!submission.empty())
        submission.pop_back();
    submission += "|";

    // ---- Launch ----
    for (int x : sorted[0].launch_indices) submission += std::to_string(x + 1) + ",";
    submission += "-1,";
    for (int x : sorted[1].launch_indices) submission += std::to_string(x + 1) + ",";
    if (!submission.empty())
        submission.pop_back();
    submission += "|";

    // ---- Land ----
    for (int x : sorted[0].land_indices) submission += std::to_string(x + 1) + ",";
    submission += "-1,";
    for (int x : sorted[1].land_indices) submission += std::to_string(x + 1) + ",";
    if (!submission.empty())
        submission.pop_back();

    return submission;
}
