#include "datahandling/convert_to_submission.h"
#include<map>
#include<algorithm>

std::string convert_to_submission(const Solution& solution) {
    std::string submission;

    // ---- Truck route ----
    for (int n : solution.truck_route)
        submission += std::to_string(n) + ",";
    submission.pop_back();
    submission += "|";

    constexpr int DRONES = 2;

    std::vector<int> deliver[DRONES];
    std::vector<int> launch[DRONES];
    std::vector<int> land[DRONES];

    // ---- Timeline scan ----
    for (const auto& [launch_node, vec] : solution.drone_map) {
        for (int d = 0; d < DRONES; ++d) {
            auto [deliver_node, land_node] = vec[d];
            if (deliver_node == -1) continue;

            deliver[d].push_back(deliver_node);   // served order
            launch[d].push_back(launch_node);
            land[d].push_back(land_node);
        }
    }

    // ---- Numeric order sections ----
    for (int d = 0; d < DRONES; ++d) {
        std::sort(launch[d].begin(), launch[d].end());
        std::sort(land[d].begin(), land[d].end());
    }

    // ---- Append helper ----
    auto append = [&](int d) {
        for (int x : deliver[d]) submission += std::to_string(x) + ",";
    };

    // Deliver
    append(0);
    submission += "-1,";
    append(1);
    submission.pop_back();
    submission += "|";

    // Launch
    for (int x : launch[0]) submission += std::to_string(x) + ",";
    submission += "-1,";
    for (int x : launch[1]) submission += std::to_string(x) + ",";
    submission.pop_back();
    submission += "|";

    // Land
    for (int x : land[0]) submission += std::to_string(x) + ",";
    submission += "-1,";
    for (int x : land[1]) submission += std::to_string(x) + ",";
    submission.pop_back();

    return submission;
}
