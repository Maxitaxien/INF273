#include "verification/solution.h"
#include "verification/feasibility_check.h"
#include "datahandling/instance.h"
#include "operators/helpers.h"
#include "operators/interval_helpers.h"
#include "general/get_truck_arrival_times.h"
#include <set>
#include <unordered_map>
#include <limits>

std::pair<bool, Solution> assign_launch_and_land_n_lookahead(const Instance &inst, Solution &sol, int idx, int new_deliver, int drone, int look_ahead)
{
    int maximum_lookahead;
    if (sol.drones[drone].launch_indices.size() < look_ahead)
    {
        maximum_lookahead = look_ahead;
    }
    else
    {
        int i = 0;
        while (i < sol.drones[drone].launch_indices.size() && sol.drones[drone].launch_indices[i] < idx)
        {
            i++;
        }
        maximum_lookahead = std::max(sol.drones[drone].launch_indices[i], look_ahead);
    }

    std::vector<long long> drone_available;
    long long total_drone_arrival;
    std::vector<long long> truck_arrival = get_truck_arrival_times(inst, sol, drone_available, total_drone_arrival);

    // Try all potential landings
    // Where best is defined as the place where the truck waits for the least amount of time
    // This might be the case for multiple landing spots - in this case, we will end up assigning to the first
    // (this is desirable to keep the drone free for future deliveries)
    long long best = std::numeric_limits<long long>::infinity();
    int optimal_land_idx = -1;

    int launch_idx = idx;
    long long launch_time_allowed = std::max(truck_arrival[launch_idx], drone_available[drone]);

    for (int land_idx = 0; land_idx < maximum_lookahead; land_idx++)
    {
        long long drone_total_duration = (launch_time_allowed - truck_arrival[launch_idx]) + inst.drone_matrix[sol.truck_route[launch_idx]][new_deliver] + inst.drone_matrix[new_deliver][sol.truck_route[land_idx]];
        if (drone_total_duration <= inst.lim)
        {
            long long truck_wait_time = drone_total_duration - (truck_arrival[land_idx] - truck_arrival[launch_idx]);
            if (truck_wait_time < best)
            {
                best = truck_wait_time;
                optimal_land_idx = land_idx;
            }
        }
    }

    // If no drone flights were under the duration, we were unsuccessfull :(
    if (optimal_land_idx == -1)
    {
        return {false, sol};
    }
    else
    {
        sol.drones[drone].launch_indices.push_back(launch_idx);
        sol.drones[drone].deliver_nodes.push_back(new_deliver);
        sol.drones[drone].land_indices.push_back(optimal_land_idx);
        return {true, sol};
    }
}

std::pair<bool, Solution> greedy_assign_launch_and_land(const Instance &instance, Solution &solution, int new_deliver, int drone)
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

    for (int l = 0; l < sorted_points.size(); l++)
    {
        for (int r = l + 1; r < sorted_points.size(); r++)
        {

            int p1 = sorted_points[l];
            int p2 = sorted_points[r];

            int pos1 = node_positions.at(p1);
            int pos2 = node_positions.at(p2);
            if (pos1 > pos2)
                std::swap(pos1, pos2);

            long long launch_time_allowed = std::max(truck_arrival[pos1], drone_available[drone]);

            long long drone_total_duration = (launch_time_allowed - truck_arrival[pos1]) + instance.drone_matrix[solution.truck_route[pos1]][new_deliver] + instance.drone_matrix[new_deliver][solution.truck_route[pos2]];

            bool overlap = overlaps(drone_intervals, pos1, pos2);

            if (drone_total_duration <= instance.lim && !overlap && pos2 < solution.truck_route.size() && pos2 < (int)solution.truck_route.size() - 1)
            {
                solution.drones[drone].launch_indices.push_back(pos1);
                solution.drones[drone].deliver_nodes.push_back(new_deliver);
                solution.drones[drone].land_indices.push_back(pos2);
                return {true, solution};
            }
        }
    }

    return {false, solution};
}

Solution &fix_feasibility_for_drone(const Instance &instance, Solution &sol, int drone)
{
    DroneCollection &c = sol.drones[drone];
    std::set<Interval> intervals = get_intervals(sol, drone);

    for (int i = 0; i < c.launch_indices.size();)
    {
        int launch_idx = c.launch_indices[i];
        int land_idx = c.land_indices[i];
        int deliver = c.deliver_nodes[i];
        bool bad = (launch_idx >= land_idx) || !specific_drone_flight_under_lim(instance, sol, drone, i);

        Interval self{launch_idx, land_idx};
        intervals.erase(self);

        if (overlaps(intervals, launch_idx, land_idx))
            bad = true;

        if (bad)
        {
            auto [success, _] = greedy_assign_launch_and_land(instance, sol, deliver, drone);
            if (!success)
            {
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

Solution simple_fix_validity(Solution &solution)
{
    int final_index = solution.truck_route.size() - 1;
    std::vector<int> to_reinsert;

    for (int c = 0; c < solution.drones.size(); c++)
    {
        DroneCollection &dc = solution.drones[c];

        for (int i = 0; i < dc.deliver_nodes.size();)
        {
            int landing = dc.land_indices[i];
            if (landing == final_index)
            {
                int deliver = dc.deliver_nodes[i];
                to_reinsert.push_back(deliver);

                dc.launch_indices.erase(dc.launch_indices.begin() + i);
                dc.land_indices.erase(dc.land_indices.begin() + i);
                dc.deliver_nodes.erase(dc.deliver_nodes.begin() + i);
            }
            else
            {
                i++;
            }
        }
    }

    for (int reinsert : to_reinsert)
    {
        solution.truck_route.push_back(reinsert);
    }

    return solution;
}

Solution fix_validity(const Instance &instance, Solution &solution, int drone)
{
    return solution;
}

Solution fix_overall_feasibility(const Instance &instance, Solution &solution)
{
    solution = fix_feasibility_for_drone(instance, solution, 0);
    solution = fix_feasibility_for_drone(instance, solution, 1);
    solution = simple_fix_validity(solution);
    return solution;
}