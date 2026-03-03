#include "verification/solution.h"
#include "verification/feasibility_check.h"
#include "datahandling/instance.h"
#include "operators/helpers.h"
#include "operators/interval_helpers.h"
#include "general/get_truck_arrival_times.h"
#include <set>
#include <unordered_map>

std::pair<bool, Solution> assign_launch_and_land(const Instance& instance, Solution& solution, int new_deliver, int drone)
{
    std::unordered_map<int, int> node_positions = get_node_positions(solution);
    std::vector<int> sorted_points = sort_by_distance_to_point(instance, solution, new_deliver);
    std::set<Interval> drone_intervals = get_intervals(solution, drone);

    std::vector<long long> drone_available;
    long long total_drone_arrival;

    std::vector<long long> truck_arrival = get_truck_arrival_times(instance, solution, drone_available, total_drone_arrival);

    // Two pointer for assignment
    if (sorted_points.size() < 2)
    return {false, solution};

    for (int l = 0; l < sorted_points.size(); l++) {
        for (int r = l + 1; r < sorted_points.size(); r++) {

            int p1 = sorted_points[l];
            int p2 = sorted_points[r];

            int pos1 = node_positions.at(p1);
            int pos2 = node_positions.at(p2);
            if (pos1 > pos2) std::swap(pos1, pos2);

            long long launch_time_allowed = std::max(truck_arrival[pos1], drone_available[drone]);

            long long drone_total_duration = (launch_time_allowed - truck_arrival[pos1])
                                            + instance.drone_matrix[solution.truck_route[pos1]][new_deliver]
                                            + instance.drone_matrix[new_deliver][solution.truck_route[pos2]];

            bool overlap = overlaps(drone_intervals, pos1, pos2);

            if (drone_total_duration <= instance.lim && !overlap && pos2 < solution.truck_route.size() && pos2 < (int)solution.truck_route.size() - 1) {
                solution.drones[drone].launch_indices.push_back(pos1);
                solution.drones[drone].deliver_nodes.push_back(new_deliver);
                solution.drones[drone].land_indices.push_back(pos2);
                return {true, solution};
            }
        }
    }

    return {false, solution};
}

Solution& fix_feasibility_for_drone(const Instance& instance, Solution& sol, int drone) {
    DroneCollection& c = sol.drones[drone];
    std::set<Interval> intervals = get_intervals(sol, drone);

    for (int i = 0; i < c.launch_indices.size(); ) {
        int launch_idx = c.launch_indices[i];
        int land_idx   = c.land_indices[i];
        int deliver    = c.deliver_nodes[i];
        bool bad = (launch_idx >= land_idx) || !specific_drone_flight_under_lim(instance, sol, drone, i);

        Interval self{launch_idx, land_idx};
        intervals.erase(self);

        if (overlaps(intervals, launch_idx, land_idx)) bad = true;

        if (bad) {
            auto [success, _] = assign_launch_and_land(instance, sol, deliver, drone);
            if (!success) {
                remove_drone_flight(sol, drone, i);
                int truck_pos = std::min(launch_idx + 1, (int)sol.truck_route.size());
                sol.truck_route.insert(sol.truck_route.begin() + truck_pos, deliver);
                intervals = get_intervals(sol, drone);
                continue;
            }
        }
        i++;
    }
    return sol;
}


Solution simple_fix_validity(Solution& solution) {
    int final_index = solution.truck_route.size() - 1;
    std::vector<int> to_reinsert;

    for (int c = 0; c < solution.drones.size(); c++) {
        DroneCollection& dc = solution.drones[c]; 
        
        for (int i = 0; i < dc.deliver_nodes.size(); ) {
            int landing = dc.land_indices[i];
            if (landing == final_index) {
                int deliver = dc.deliver_nodes[i];
                to_reinsert.push_back(deliver);

                dc.launch_indices.erase(dc.launch_indices.begin() + i);
                dc.land_indices.erase(dc.land_indices.begin() + i);
                dc.deliver_nodes.erase(dc.deliver_nodes.begin() + i);
            } else {
                i++;
            }
        }
}
    
    for (int reinsert : to_reinsert) {
        solution.truck_route.push_back(reinsert);
    }

    return solution;
}

Solution fix_validity(const Instance& instance, Solution& solution, int drone) {
    return solution;
}


Solution fix_overall_feasibility(const Instance& instance, Solution& solution) {
    solution = fix_feasibility_for_drone(instance, solution, 0);
    solution = fix_feasibility_for_drone(instance, solution, 1);
    solution = simple_fix_validity(solution);
    return solution;
}