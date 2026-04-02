#include "operators/drone_rendezvous_shift.h"
#include "general/random.h"
#include "general/sort_drone_collection.h"
#include "operators/interval_helpers.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <algorithm>
#include <limits>
#include <set>
#include <utility>
#include <vector>

namespace
{
bool valid_drone_and_flight(const Solution &sol, int drone, int flight_idx)
{
    if (drone < 0 || drone >= (int)(sol.drones.size()))
    {
        return false;
    }

    const DroneCollection &collection = sol.drones[drone];
    const int flight_count = std::min({
        (int)(collection.launch_indices.size()),
        (int)(collection.land_indices.size()),
        (int)(collection.deliver_nodes.size())});
    return flight_idx >= 0 && flight_idx < flight_count;
}

bool can_place_without_overlap(
    const std::set<Interval> &intervals,
    int launch_idx,
    int land_idx)
{
    if (launch_idx >= land_idx)
    {
        return false;
    }

    return !overlaps(intervals, launch_idx, land_idx);
}
}

bool drone_rendezvous_shift(
    const Instance &inst,
    Solution &sol,
    int drone,
    int flight_idx,
    int launch_window,
    int land_window)
{
    if (!valid_drone_and_flight(sol, drone, flight_idx))
    {
        return false;
    }

    const int route_size = (int)(sol.truck_route.size());
    if (route_size < 2)
    {
        return false;
    }

    const DroneCollection &collection = sol.drones[drone];
    const int current_launch = collection.launch_indices[flight_idx];
    const int current_land = collection.land_indices[flight_idx];
    if (current_launch < 0 || current_land < 0 ||
        current_launch >= route_size || current_land >= route_size ||
        current_launch >= current_land)
    {
        return false;
    }

    const int safe_launch_window = std::max(0, launch_window);
    const int safe_land_window = std::max(0, land_window);
    std::set<Interval> occupied = get_intervals(sol, drone);
    occupied.erase(Interval{current_launch, current_land});

    const int min_launch = std::max(0, current_launch - safe_launch_window);
    const int max_launch = std::min(route_size - 2, current_launch + safe_launch_window);
    const int min_land = std::max(1, current_land - safe_land_window);
    const int max_land = std::min(route_size - 1, current_land + safe_land_window);

    bool found = false;
    long long best_cost = std::numeric_limits<long long>::max();
    Solution best_solution;

    for (int new_launch = min_launch; new_launch <= max_launch; ++new_launch)
    {
        for (int new_land = std::max(new_launch + 1, min_land); new_land <= max_land; ++new_land)
        {
            if (new_launch == current_launch && new_land == current_land)
            {
                continue;
            }

            if (!can_place_without_overlap(occupied, new_launch, new_land))
            {
                continue;
            }

            Solution candidate = sol;
            candidate.drones[drone].launch_indices[flight_idx] = new_launch;
            candidate.drones[drone].land_indices[flight_idx] = new_land;
            if ((int)(candidate.drones[drone].launch_indices.size()) > 1)
            {
                sort_drone_collection(candidate.drones[drone]);
            }

            if (!master_check(inst, candidate, false))
            {
                continue;
            }

            const long long candidate_cost = objective_function_impl(inst, candidate);
            if (!found || candidate_cost < best_cost)
            {
                best_cost = candidate_cost;
                best_solution = std::move(candidate);
                found = true;
            }
        }
    }

    if (!found)
    {
        return false;
    }

    sol = std::move(best_solution);
    return true;
}

bool drone_rendezvous_shift_first_improvement(
    const Instance &inst,
    Solution &sol)
{
    std::vector<std::pair<int, int>> flights;
    for (int drone = 0; drone < (int)(sol.drones.size()); ++drone)
    {
        const DroneCollection &collection = sol.drones[drone];
        const int flight_count = std::min({
            (int)(collection.launch_indices.size()),
            (int)(collection.land_indices.size()),
            (int)(collection.deliver_nodes.size())});
        for (int flight_idx = 0; flight_idx < flight_count; ++flight_idx)
        {
            flights.emplace_back(drone, flight_idx);
        }
    }

    if (flights.empty())
    {
        return false;
    }

    std::shuffle(flights.begin(), flights.end(), gen);
    const int window = 2;

    for (const auto &[drone, flight_idx] : flights)
    {
        Solution candidate = sol;
        if (!drone_rendezvous_shift(inst, candidate, drone, flight_idx, window, window))
        {
            continue;
        }

        sol = std::move(candidate);
        return true;
    }

    return false;
}
