#include "datahandling/instance_preprocessing.h"
#include "general/random.h"
#include "operators/candidate_evaluation.h"
#include "verification/objective_value.h"

#include <algorithm>
#include <limits>
#include <random>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

bool contains_node(const std::vector<int> &xs, int x)
{
    return std::find(xs.begin(), xs.end(), x) != xs.end();
}

void add_unique(std::vector<int> &xs, int x)
{
    add_unique_int(xs, x);
}

bool flight_touches_removed_truck_index(
    int launch_idx,
    int land_idx,
    int removed_idx)
{
    return launch_idx == removed_idx || land_idx == removed_idx;
}

Solution remove_one_truck_customer_and_invalidated_flights(
    const Solution &sol,
    int removed_truck_idx,
    std::vector<int> &repair_pool)
{
    Solution partial;
    partial.truck_route.reserve(sol.truck_route.size() - 1);

    const int removed_customer = sol.truck_route[removed_truck_idx];
    add_unique(repair_pool, removed_customer);

    std::vector<int> old_to_new(sol.truck_route.size(), -1);

    for (int old_idx = 0; old_idx < (int)sol.truck_route.size(); ++old_idx)
    {
        if (old_idx == removed_truck_idx)
            continue;

        old_to_new[old_idx] = (int)partial.truck_route.size();
        partial.truck_route.push_back(sol.truck_route[old_idx]);
    }

    partial.drones.resize(sol.drones.size());

    for (int d = 0; d < (int)sol.drones.size(); ++d)
    {
        const DroneCollection &src = sol.drones[d];
        DroneCollection &dst = partial.drones[d];

        for (int t = 0; t < (int)src.deliver_nodes.size(); ++t)
        {
            const int customer = src.deliver_nodes[t];
            const int old_launch = src.launch_indices[t];
            const int old_land = src.land_indices[t];

            const bool old_terminal =
                old_land == (int)sol.truck_route.size();

            if (flight_touches_removed_truck_index(
                    old_launch,
                    old_land,
                    removed_truck_idx))
            {
                add_unique(repair_pool, customer);
                continue;
            }

            dst.launch_indices.push_back(old_to_new[old_launch]);
            dst.deliver_nodes.push_back(customer);

            if (old_terminal)
                dst.land_indices.push_back((int)partial.truck_route.size());
            else
                dst.land_indices.push_back(old_to_new[old_land]);
        }
    }

    return partial;
}

bool greedy_repair_one_customer(
    const Instance &inst,
    Solution &partial,
    int customer)
{
    long long best_cost = std::numeric_limits<long long>::max();
    Solution best_solution;
    bool found = false;

    const int route_size = (int)partial.truck_route.size();

    for (int insert_pos = 1; insert_pos <= route_size; ++insert_pos)
    {
        Solution candidate = partial;
        insert_truck_customer_and_shift_drones(candidate, insert_pos, customer);

        long long cost = 0;
        Solution canonical_candidate;

        if (evaluate_candidate_with_timing(
                inst,
                std::move(candidate),
                cost,
                canonical_candidate) &&
            cost < best_cost)
        {
            best_cost = cost;
            best_solution = std::move(canonical_candidate);
            found = true;
        }
    }

    for (int launch_idx = 0; launch_idx < route_size; ++launch_idx)
    {
        const int launch_node = partial.truck_route[launch_idx];

        for (int land_idx = launch_idx + 1; land_idx <= route_size; ++land_idx)
        {
            const bool terminal_depot = land_idx == route_size;
            const int land_node =
                terminal_depot ? 0 : partial.truck_route[land_idx];

            if (!pure_drone_flight_within_limit(
                    inst,
                    launch_node,
                    customer,
                    land_node))
                continue;

            for (int drone = 0; drone < inst.m; ++drone)
            {
                Solution candidate = partial;

                append_direct_drone_flight(
                    candidate,
                    drone,
                    launch_idx,
                    customer,
                    land_idx);

                long long cost = 0;
                Solution canonical_candidate;

                if (evaluate_candidate_with_timing(
                        inst,
                        std::move(candidate),
                        cost,
                        canonical_candidate) &&
                    cost < best_cost)
                {
                    best_cost = cost;
                    best_solution = std::move(canonical_candidate);
                    found = true;
                }
            }
        }
    }

    if (!found)
        return false;

    partial = std::move(best_solution);
    return true;
}

bool greedy_repair(
    const Instance &inst,
    Solution &partial,
    std::vector<int> repair_pool)
{
    random_shuffle(repair_pool);

    while (!repair_pool.empty())
    {
        long long best_global_cost = std::numeric_limits<long long>::max();
        int best_customer_idx = -1;
        Solution best_after_insert;

        for (int idx = 0; idx < (int)repair_pool.size(); ++idx)
        {
            Solution trial = partial;

            if (!greedy_repair_one_customer(inst, trial, repair_pool[idx]))
                continue;

            long long cost = 0;
            Solution canonical_trial;

            if (!evaluate_candidate_with_timing(
                    inst,
                    std::move(trial),
                    cost,
                    canonical_trial))
                continue;

            if (cost < best_global_cost)
            {
                best_global_cost = cost;
                best_customer_idx = idx;
                best_after_insert = std::move(canonical_trial);
            }
        }

        if (best_customer_idx < 0)
            return false;

        partial = std::move(best_after_insert);
        repair_pool.erase(repair_pool.begin() + best_customer_idx);
    }

    return true;
}

} // namespace


bool truck_relocate_1_random(const Instance &inst, Solution &sol)
{
    const int route_size = (int)sol.truck_route.size();

    if (route_size <= 2)
        return false;

    const long long old_cost = objective_function_impl(inst, sol);

    std::vector<int> removal_indices(route_size - 1);
    std::iota(removal_indices.begin(), removal_indices.end(), 1);
    random_shuffle(removal_indices);

    for (int removed_idx : removal_indices)
    {
        std::vector<int> repair_pool;

        Solution partial =
            remove_one_truck_customer_and_invalidated_flights(
                sol,
                removed_idx,
                repair_pool);

        if (repair_pool.empty())
            continue;

        if (!greedy_repair(inst, partial, repair_pool))
            continue;

        long long new_cost = 0;
        Solution canonical_candidate;

        if (!evaluate_candidate_with_timing(
                inst,
                std::move(partial),
                new_cost,
                canonical_candidate))
            continue;

        if (new_cost < old_cost)
        {
            sol = std::move(canonical_candidate);
            return true;
        }
    }

    return false;
}
