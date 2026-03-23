#include "operators/solution_fixers.h"
#include "verification/solution.h"
#include "verification/feasibility_check.h"
#include "datahandling/instance.h"
#include "operators/helpers.h"
#include "operators/interval_helpers.h"
#include "general/get_truck_arrival_times.h"
#include "operators/drone_planner.h"
#include <algorithm>
#include <set>
#include <unordered_map>
#include <limits>

std::pair<bool, Solution> assign_launch_and_land_n_lookahead(const Instance &inst, Solution &sol, int idx, int new_deliver, int drone, int look_ahead)
{
    // Guard against invalid inputs (common when operators mutate route/drone tables)
    if (drone < 0 || drone >= (int)sol.drones.size())
        return {false, sol};
    if (idx < 0 || idx >= (int)sol.truck_route.size())
        return {false, sol};

    // Avoid assigning a delivery that is already on the truck route (prevents duplicates)
    if (std::find(sol.truck_route.begin(), sol.truck_route.end(), new_deliver) != sol.truck_route.end())
        return {false, sol};

    simple_fix_validity(sol);
    std::set<Interval> drone_intervals = get_intervals(sol, drone);

    int maximum_lookahead;
    // estimate how many positions ahead to consider; if there are existing launch
    // indices we don't want to overlap them, otherwise just use the lookahead
    if ((int)(sol.drones[drone].launch_indices.size()) < look_ahead)
    {
        maximum_lookahead = look_ahead;
    }
    else
    {
        int i = 0;
        while (i < (int)sol.drones[drone].launch_indices.size() && sol.drones[drone].launch_indices[i] < idx)
        {
            i++;
        }
        // clamp at next existing launch (or look_ahead, whichever is smaller)
        if (i == (int)(sol.drones[drone].launch_indices.size()))
            maximum_lookahead = look_ahead;
        else
            maximum_lookahead = std::min(sol.drones[drone].launch_indices[i] - idx, look_ahead);
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

    // search forward from the launch position; `maximum_lookahead` is a count, not
    // an absolute index.  clamp it so we never go past the end of the route.
    int max_offset = std::min(maximum_lookahead, (int)sol.truck_route.size() - launch_idx - 1);
    for (int offset = 1; offset <= max_offset; ++offset)
    {
        int land_idx = launch_idx + offset;

        // Never land at the final stop, as it's treated as the depot-return placeholder.
        if (land_idx >= (int)sol.truck_route.size() - 1)
            continue;

        // Prevent overlapping an existing drone flight for this drone.
        if (overlaps(drone_intervals, launch_idx, land_idx))
            continue;

        long long drone_total_duration = (launch_time_allowed - truck_arrival[launch_idx]) +
                                         inst.drone_matrix[sol.truck_route[launch_idx]][new_deliver] +
                                         inst.drone_matrix[new_deliver][sol.truck_route[land_idx]];
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
    // We want to pick a launch/land pair (indices in the truck route) that:
    //  - keeps the flight under the time limit
    //  - does not overlap any existing flight of this drone
    //  - does not land at the final truck stop (invalid in this model)
    //  - does not attempt to deliver to a node that is already on the truck route

    simple_fix_validity(solution);
    if (std::find(solution.truck_route.begin(), solution.truck_route.end(), new_deliver) != solution.truck_route.end())
        return {false, solution};

    std::set<Interval> drone_intervals = get_intervals(solution, drone);

    std::vector<long long> drone_available;
    long long total_drone_arrival;
    std::vector<long long> truck_arrival = get_truck_arrival_times(instance, solution, drone_available, total_drone_arrival);

    const int route_size = (int)solution.truck_route.size();
    if (route_size < 3)
        return {false, solution};

    // Build a list of candidate indices (excluding depot at index 0, and excluding final index).
    // Also exclude any position where the truck already visits the delivery node.
    std::vector<int> candidate_indices;
    candidate_indices.reserve(route_size);

    for (int idx = 1; idx < route_size - 1; ++idx)
    {
        if (solution.truck_route[idx] == new_deliver)
            continue;
        candidate_indices.push_back(idx);
    }

    if (candidate_indices.size() < 2)
        return {false, solution};

    // Sort candidates by drone distance from their node to the delivery node.
    std::sort(candidate_indices.begin(), candidate_indices.end(),
              [&](int a, int b) {
                  return instance.drone_matrix[solution.truck_route[a]][new_deliver] <
                         instance.drone_matrix[solution.truck_route[b]][new_deliver];
              });

    long long best_wait = std::numeric_limits<long long>::infinity();
    int best_launch = -1;
    int best_land = -1;

    for (int i = 0; i < (int)candidate_indices.size(); ++i)
    {
        for (int j = i + 1; j < (int)candidate_indices.size(); ++j)
        {
            int launch_idx = candidate_indices[i];
            int land_idx = candidate_indices[j];

            // Ensure the land event is not at the final truck stop.
            if (land_idx >= route_size - 1)
                continue;

            long long launch_time_allowed = std::max(truck_arrival[launch_idx], drone_available[drone]);

            long long drone_total_duration = (launch_time_allowed - truck_arrival[launch_idx]) +
                                             instance.drone_matrix[solution.truck_route[launch_idx]][new_deliver] +
                                             instance.drone_matrix[new_deliver][solution.truck_route[land_idx]];

            if (drone_total_duration > instance.lim)
                continue;

            if (overlaps(drone_intervals, launch_idx, land_idx))
                continue;

            long long truck_wait_time = drone_total_duration - (truck_arrival[land_idx] - truck_arrival[launch_idx]);
            if (truck_wait_time < best_wait)
            {
                best_wait = truck_wait_time;
                best_launch = launch_idx;
                best_land = land_idx;
            }
        }
    }

    if (best_launch == -1 || best_land == -1)
        return {false, solution};

    solution.drones[drone].launch_indices.push_back(best_launch);
    solution.drones[drone].deliver_nodes.push_back(new_deliver);
    solution.drones[drone].land_indices.push_back(best_land);

    return {true, solution};
}

Solution &fix_feasibility_for_drone(const Instance &instance, Solution &sol, int drone)
{
    DroneCollection &c = sol.drones[drone];
    std::set<Interval> intervals = get_intervals(sol, drone);

    for (int i = 0; i < (int)(c.launch_indices.size());)
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

Solution &fix_feasibility_for_drone_alternative(const Instance &instance,
                                                Solution &sol, int drone)
{
    DroneCollection &c = sol.drones[drone];
    std::set<Interval> intervals = get_intervals(sol, drone);

    for (int i = 0; i < (int)(c.launch_indices.size()); ++i)
    {
        int launch = c.launch_indices[i];
        int land = c.land_indices[i];
        bool bad = (launch >= land) ||
                   !specific_drone_flight_under_lim(instance, sol, drone, i) ||
                   overlaps(intervals, launch, land);
        if (!bad)
            continue;

        auto [ok, newsol] = greedy_assign_launch_and_land(instance, sol, c.deliver_nodes[i], drone);
        if (!ok)
        {
            remove_drone_flight(sol, drone, i);
            sol.truck_route.insert(sol.truck_route.begin() + std::min(launch + 1, (int)sol.truck_route.size()),
                                   c.deliver_nodes[i]);
        }
        // we've modified the structure; give caller a fresh check
        return fix_feasibility_for_drone_alternative(instance, sol, drone);
    }
    return sol;
}

Solution simple_fix_validity(Solution &solution)
{
    const int route_size = (int)(solution.truck_route.size());
    const int final_index = route_size - 1;
    std::vector<int> to_reinsert;

    for (int c = 0; c < (int)(solution.drones.size()); c++)
    {
        DroneCollection &dc = solution.drones[c];

        for (int i = 0; i < (int)(dc.deliver_nodes.size());)
        {
            const int launch = dc.launch_indices[i];
            const int landing = dc.land_indices[i];
            const bool invalid_index =
                launch < 0 || landing < 0 ||
                launch >= route_size || landing >= route_size ||
                launch >= landing || landing >= final_index;

            if (invalid_index)
            {
                to_reinsert.push_back(dc.deliver_nodes[i]);
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
    (void)instance;
    (void)drone;
    return solution;
}

Solution fix_overall_feasibility(const Instance &instance, Solution &solution)
{
    solution = simple_fix_validity(solution);
    if (master_check(instance, solution, false))
    {
        return solution;
    }

    auto [planner_obj, planned_solution] = drone_planner(instance, solution);
    (void)planner_obj;

    if (master_check(instance, planned_solution, false))
    {
        return planned_solution;
    }

    solution = fix_feasibility_for_drone(instance, solution, 0);
    solution = fix_feasibility_for_drone(instance, solution, 1);
    solution = simple_fix_validity(solution);
    return solution;
}






