 #include "datahandling/convert_to_submission.h"
 #include <map>
 #include <algorithm>
 #include <numeric>
 
 std::string convert_to_submission(const Solution& solution) {
    std::string submission;
    
    // ---- Truck route ----
    for (int n : solution.truck_route) {
        submission += std::to_string(n) + ",";
    }
    submission.pop_back();
    submission += "|";

    // ----- Truck timeline ----
    std::unordered_map<int, int> truck_pos;
    for (int i = 0; i < solution.truck_route.size() - 1; ++i)
        truck_pos[solution.truck_route[i]] = i;
    
    constexpr int DRONES = 2;
    
    std::vector<std::pair<int,int>> deliveries[DRONES];
    std::vector<int> launch[DRONES];
    std::vector<int> land[DRONES];
    

    // Prepare deliveries, launch, land per drone
    for (const auto& [launch_node, vec] : solution.drone_map) {
        for (int d = 0; d < DRONES; ++d) {
            auto [deliver_node, land_node] = vec[d];
            if (deliver_node == -1) continue;

            int launch_idx = truck_pos[launch_node]; // 0-based
            int land_idx   = truck_pos[land_node];   // 0-based

            deliveries[d].push_back({launch_idx, deliver_node});
            launch[d].push_back(launch_idx);         // store 0-based index
            land[d].push_back(land_idx);            // store 0-based index
        }
    }

    // Sort deliveries by launch index
    for (int d = 0; d < DRONES; ++d) {
        std::vector<int> sorted_launch, sorted_land, sorted_deliver;
        
        // sort trips by launch index
        std::vector<std::tuple<int,int,int>> trips;
        for (size_t i = 0; i < deliveries[d].size(); ++i)
            trips.emplace_back(launch[d][i], land[d][i], deliveries[d][i].second);
        
        std::sort(trips.begin(), trips.end()); // sorts by launch index

        // unpack back to vectors
        launch[d].clear(); land[d].clear(); deliveries[d].clear();
        for (auto& [l_idx, r_idx, node] : trips) {
            launch[d].push_back(l_idx);
            land[d].push_back(r_idx);
            deliveries[d].push_back({l_idx, node});
        }
    }

    // ---- Append helper for deliveries ----
    auto append_deliveries = [&](int d){
        for (auto& [_, node] : deliveries[d])
            submission += std::to_string(node) + ",";
    };

    
    // Deliver
    append_deliveries(0);
    submission += "-1,";
    append_deliveries(1);
    submission.pop_back();
    submission += "|";
    
    // Launch (Note 1-indexing)
    for (int x : launch[0]) submission += std::to_string(x + 1) + ",";
    submission += "-1,";
    for (int x : launch[1]) submission += std::to_string(x + 1) + ",";
    submission.pop_back();
    submission += "|";

    // Land
    for (int x : land[0]) submission += std::to_string(x + 1) + ",";
    submission += "-1,";
    for (int x : land[1]) submission += std::to_string(x + 1) + ",";
    submission.pop_back();
    
    return submission;
 }