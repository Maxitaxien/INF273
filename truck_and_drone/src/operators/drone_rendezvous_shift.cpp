#include "operators/drone_rendezvous_shift.h"
#include "general/random.h"
#include "general/roulette_wheel_selection.h"
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
constexpr size_t kTopShiftCandidates = 5;

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

bool find_best_shift_for_flight(
    const Instance &inst,
    const Solution &sol,
    int drone,
    int flight_idx,
    int launch_window,
    int land_window,
    Solution &best_solution_out,
    long long &best_cost_out)
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

    best_cost_out = best_cost;
    best_solution_out = std::move(best_solution);
    return true;
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
    Solution best_solution;
    long long ignored_cost = 0;
    if (!find_best_shift_for_flight(
            inst,
            sol,
            drone,
            flight_idx,
            launch_window,
            land_window,
            best_solution,
            ignored_cost))
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

bool drone_rendezvous_shift_best_improvement(
    const Instance &inst,
    Solution &sol)
{
    const int window = 3;

    std::vector<std::pair<long long, Solution>> top_candidates;
    top_candidates.reserve(kTopShiftCandidates);

    for (int drone = 0; drone < (int)(sol.drones.size()); ++drone)
    {
        const DroneCollection &collection = sol.drones[drone];
        const int flight_count = std::min({
            (int)(collection.launch_indices.size()),
            (int)(collection.land_indices.size()),
            (int)(collection.deliver_nodes.size())});

        for (int flight_idx = 0; flight_idx < flight_count; ++flight_idx)
        {
            Solution candidate;
            long long candidate_cost = 0;
            if (!find_best_shift_for_flight(
                    inst,
                    sol,
                    drone,
                    flight_idx,
                    window,
                    window,
                    candidate,
                    candidate_cost))
            {
                continue;
            }

            auto insert_it = top_candidates.begin();
            while (insert_it != top_candidates.end() &&
                   insert_it->first <= candidate_cost)
            {
                ++insert_it;
            }

            top_candidates.insert(
                insert_it,
                std::make_pair(candidate_cost, std::move(candidate)));

            if (top_candidates.size() > kTopShiftCandidates)
            {
                top_candidates.pop_back();
            }
        }
    }

    if (top_candidates.empty())
    {
        return false;
    }

    const int selected_idx = roulette_wheel_selection_exponential(
        (int)top_candidates.size());
    sol = std::move(top_candidates[(size_t)selected_idx].second);
    return true;
}
