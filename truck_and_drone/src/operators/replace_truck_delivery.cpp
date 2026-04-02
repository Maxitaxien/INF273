#include "operators/replace_truck_delivery.h"
#include "datahandling/instance.h"
#include "general/sort_drone_collection.h"
#include "operators/helpers.h"
#include "solution_fixers/solution_fixers.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <algorithm>
#include <limits>
#include <random>
#include <utility>

extern std::mt19937 gen;

namespace
{
void shift_drone_indices_after_truck_change(Solution &sol, int idx, int delta)
{
    for (DroneCollection &drone_collection : sol.drones)
    {
        const int flight_count = (int)(drone_collection.launch_indices.size());
        for (int i = 0; i < flight_count; ++i)
        {
            if (drone_collection.launch_indices[i] >= idx)
            {
                drone_collection.launch_indices[i] += delta;
            }

            if (drone_collection.land_indices[i] >= idx)
            {
                drone_collection.land_indices[i] += delta;
            }
        }
    }
}
}

bool replace_truck_delivery(const Instance &inst, Solution &sol, int idx, int drone)
{
    if (idx <= 0 || idx >= (int)(sol.truck_route.size()))
    {
        return false;
    }
    if (drone < 0 || drone >= (int)(sol.drones.size()))
    {
        return false;
    }

    int customer = sol.truck_route[idx];

    // 1: POP
    pop_truck_delivery(sol, idx);
    shift_drone_indices_after_truck_change(sol, idx, -1);

    // 3: INSERT - n = problem size / 10 index lookahead, pick best
    const int look_ahead = std::max(1, inst.n / 10);
    sort_drone_collection(sol.drones[drone]);

    auto [success, ignored_solution] = assign_launch_and_land_n_lookahead_assume_valid(
        inst,
        sol,
        idx - 1,
        customer,
        drone,
        look_ahead);
    (void)ignored_solution;
    if (!success || !master_check(inst, sol, false))
    {
        if (success)
        {
            remove_drone_flight(sol, drone);
        }
        shift_drone_indices_after_truck_change(sol, idx, 1);
        insert_truck_delivery(sol, customer, idx);
        return false;
    }

    return true;
}

bool replace_truck_delivery_random(const Instance &instance, Solution &sol)
{
    std::vector<std::pair<int, int>> candidates;
    for (int drone = 0; drone < (int)(sol.drones.size()); ++drone)
    {
        for (int idx = 1; idx < (int)(sol.truck_route.size()); ++idx)
        {
            candidates.emplace_back(drone, idx);
        }
    }

    if (candidates.empty())
    {
        return false;
    }

    std::shuffle(candidates.begin(), candidates.end(), gen);
    const int max_attempts = std::min(10, (int)(candidates.size()));
    Solution candidate = sol;

    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
        const auto [drone, idx] = candidates[attempt];
        candidate = sol;
        if (replace_truck_delivery(instance, candidate, idx, drone))
        {
            sol = std::move(candidate);
            return true;
        }
    }

    return false;
}

bool replace_truck_delivery_greedy(const Instance &instance, Solution &sol)
{
    bool success = false;
    long long best_cost = std::numeric_limits<long long>::max();
    Solution best_solution;
    Solution candidate = sol;

    for (int drone = 0; drone < (int)sol.drones.size(); ++drone)
    {
        for (int i = 1; i < (int)sol.truck_route.size(); ++i)
        {
            candidate = sol;
            if (!replace_truck_delivery(instance, candidate, i, drone))
            {
                continue;
            }

            const long long cost = objective_function_impl(instance, candidate);
            if (cost < best_cost)
            {
                best_cost = cost;
                best_solution = std::move(candidate);
                success = true;
            }
        }
    }

    if (success)
    {
        sol = std::move(best_solution);
    }
    return success;
}

