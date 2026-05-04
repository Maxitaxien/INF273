#include "operators/shaw_removal_greedy_repair.h"

#include "datahandling/instance_preprocessing.h"
#include "general/random.h"
#include "operators/route_timing.h"
#include "verification/objective_value.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
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
    if (!contains_node(xs, x))
        xs.push_back(x);
}

long long objective_from_timing(const RouteTiming &timing)
{
    long long total_time = timing.total_drone_arrival;

    for (int idx = 1; idx < (int)timing.truck_arrival.size(); ++idx)
        total_time += timing.truck_arrival[idx];

    return total_time / 100;
}

bool canonical_drone_schedule_consistent(const Solution &solution)
{
    const int route_size = (int)solution.truck_route.size();

    for (const DroneCollection &collection : solution.drones)
    {
        if (collection.launch_indices.size() != collection.land_indices.size() ||
            collection.launch_indices.size() != collection.deliver_nodes.size())
            return false;

        std::vector<std::pair<int, int>> flights;
        flights.reserve(collection.launch_indices.size());

        int last_flight_idx = -1;

        for (int idx = 0; idx < (int)collection.launch_indices.size(); ++idx)
        {
            if (last_flight_idx < 0 ||
                std::pair{
                    collection.launch_indices[idx],
                    collection.land_indices[idx]} >
                    std::pair{
                    collection.launch_indices[last_flight_idx],
                    collection.land_indices[last_flight_idx]})
            {
                last_flight_idx = idx;
            }
        }

        for (int idx = 0; idx < (int)collection.launch_indices.size(); ++idx)
        {
            const int launch_idx = collection.launch_indices[idx];
            const int land_idx = collection.land_indices[idx];
            const bool terminal_depot =
                is_terminal_depot_landing(solution, land_idx);

            if (launch_idx < 0 || launch_idx >= route_size ||
                land_idx < 0 || land_idx > route_size)
                return false;

            if (land_idx <= launch_idx)
                return false;

            if (terminal_depot && idx != last_flight_idx)
                return false;

            flights.emplace_back(launch_idx, land_idx);
        }

        std::sort(flights.begin(), flights.end());

        for (int idx = 1; idx < (int)flights.size(); ++idx)
        {
            if (flights[idx].first < flights[idx - 1].second)
                return false;
        }
    }

    return true;
}

bool evaluate_candidate(
    const Instance &inst,
    Solution candidate,
    long long &cost,
    Solution &canonical_candidate)
{
    canonical_candidate =
        canonicalize_terminal_depot_landings(inst, std::move(candidate));

    if (!canonical_drone_schedule_consistent(canonical_candidate))
        return false;

    const RouteTiming timing =
        compute_route_timing_from_canonical_solution(inst, canonical_candidate);

    for (int drone = 0; drone < (int)canonical_candidate.drones.size(); ++drone)
    {
        const DroneCollection &collection = canonical_candidate.drones[drone];

        for (int flight_idx = 0;
             flight_idx < (int)collection.launch_indices.size();
             ++flight_idx)
        {
            const int launch_idx = collection.launch_indices[flight_idx];
            const int land_idx = collection.land_indices[flight_idx];
            const int customer = collection.deliver_nodes[flight_idx];

            if (customer <= 0 || customer >= inst.n)
                return false;

            const bool terminal_depot =
                is_terminal_depot_landing(canonical_candidate, land_idx);

            const int launch_node =
                canonical_candidate.truck_route[launch_idx];

            const int land_node =
                terminal_depot ? 0 : canonical_candidate.truck_route[land_idx];

            const long long launch_time = std::max(
                timing.truck_arrival[launch_idx],
                timing.drone_ready_at_stop[drone][launch_idx]);

            const long long out_time =
                inst.drone_matrix[launch_node][customer];

            const long long back_time =
                inst.drone_matrix[customer][land_node];

            const long long drone_return =
                launch_time + out_time + back_time;

            const long long drone_wait = terminal_depot
                ? 0LL
                : std::max(timing.truck_arrival[land_idx] - drone_return, 0LL);

            if (out_time + back_time + drone_wait > inst.lim)
                return false;
        }
    }

    cost = objective_from_timing(timing);
    return true;
}

std::vector<int> all_served_customers(const Solution &sol)
{
    std::vector<int> customers;

    for (int idx = 1; idx < (int)sol.truck_route.size(); ++idx)
        add_unique(customers, sol.truck_route[idx]);

    for (const DroneCollection &dc : sol.drones)
        for (int c : dc.deliver_nodes)
            add_unique(customers, c);

    return customers;
}

int approximate_position(const Solution &sol, int customer)
{
    for (int idx = 1; idx < (int)sol.truck_route.size(); ++idx)
        if (sol.truck_route[idx] == customer)
            return idx;

    for (const DroneCollection &dc : sol.drones)
    {
        for (int t = 0; t < (int)dc.deliver_nodes.size(); ++t)
        {
            if (dc.deliver_nodes[t] == customer)
                return (dc.launch_indices[t] + dc.land_indices[t]) / 2;
        }
    }

    return 0;
}

long long shaw_relatedness(
    const Instance &inst,
    const Solution &sol,
    int a,
    int b)
{
    const long long dist =
        std::min(inst.truck_matrix[a][b], inst.truck_matrix[b][a]);

    const long long pos_penalty =
        1000LL * std::llabs(
            approximate_position(sol, a) - approximate_position(sol, b));

    return dist + pos_penalty;
}

std::vector<int> choose_shaw_removal_set(
    const Instance &inst,
    const Solution &sol,
    int remove_count)
{
    std::vector<int> candidates = all_served_customers(sol);

    if (candidates.empty())
        return {};

    remove_count = std::min(remove_count, (int)candidates.size());

    std::uniform_int_distribution<int> seed_dist(0, (int)candidates.size() - 1);
    std::vector<int> removed;
    removed.push_back(candidates[seed_dist(gen)]);

    constexpr double randomness_power = 3.0;

    while ((int)removed.size() < remove_count)
    {
        std::vector<int> remaining;

        for (int c : candidates)
            if (!contains_node(removed, c))
                remaining.push_back(c);

        if (remaining.empty())
            break;

        std::uniform_int_distribution<int> ref_dist(0, (int)removed.size() - 1);
        const int reference = removed[ref_dist(gen)];

        std::sort(
            remaining.begin(),
            remaining.end(),
            [&](int lhs, int rhs) {
                return shaw_relatedness(inst, sol, reference, lhs) <
                       shaw_relatedness(inst, sol, reference, rhs);
            });

        std::uniform_real_distribution<double> u01(0.0, 1.0);
        const int chosen_rank = std::min(
            (int)remaining.size() - 1,
            (int)std::floor(
                std::pow(u01(gen), randomness_power) * remaining.size()));

        removed.push_back(remaining[chosen_rank]);
    }

    return removed;
}

bool is_removed(const std::unordered_set<int> &removed, int customer)
{
    return removed.find(customer) != removed.end();
}

Solution remove_customers_and_invalidated_flights(
    const Solution &sol,
    const std::vector<int> &initial_removed,
    std::vector<int> &repair_pool)
{
    std::unordered_set<int> removed_set(
        initial_removed.begin(),
        initial_removed.end());

    repair_pool = initial_removed;

    std::vector<int> old_to_new(sol.truck_route.size(), -1);
    Solution partial;
    partial.truck_route.reserve(sol.truck_route.size());

    for (int old_idx = 0; old_idx < (int)sol.truck_route.size(); ++old_idx)
    {
        const int node = sol.truck_route[old_idx];

        if (old_idx > 0 && is_removed(removed_set, node))
            continue;

        old_to_new[old_idx] = (int)partial.truck_route.size();
        partial.truck_route.push_back(node);
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

            const bool removed_delivery =
                is_removed(removed_set, customer);

            const bool launch_removed =
                old_launch < 0 ||
                old_launch >= (int)old_to_new.size() ||
                old_to_new[old_launch] < 0;

            const bool old_terminal =
                old_land == (int)sol.truck_route.size();

            const bool land_removed =
                !old_terminal &&
                (old_land < 0 ||
                 old_land >= (int)old_to_new.size() ||
                 old_to_new[old_land] < 0);

            if (removed_delivery || launch_removed || land_removed)
            {
                add_unique(repair_pool, customer);
                removed_set.insert(customer);
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

void insert_truck_customer(Solution &sol, int insert_pos, int customer)
{
    sol.truck_route.insert(sol.truck_route.begin() + insert_pos, customer);

    for (DroneCollection &dc : sol.drones)
    {
        for (int &idx : dc.launch_indices)
            if (idx >= insert_pos)
                ++idx;

        for (int &idx : dc.land_indices)
            if (idx >= insert_pos)
                ++idx;
    }
}

void append_drone_flight(
    Solution &sol,
    int drone,
    int launch_idx,
    int customer,
    int land_idx)
{
    if ((int)sol.drones.size() <= drone)
        sol.drones.resize(drone + 1);

    sol.drones[drone].launch_indices.push_back(launch_idx);
    sol.drones[drone].deliver_nodes.push_back(customer);
    sol.drones[drone].land_indices.push_back(land_idx);
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
        insert_truck_customer(candidate, insert_pos, customer);

        long long cost = 0;
        Solution canonical_candidate;

        if (evaluate_candidate(inst, std::move(candidate), cost, canonical_candidate) &&
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
                append_drone_flight(
                    candidate,
                    drone,
                    launch_idx,
                    customer,
                    land_idx);

                long long cost = 0;
                Solution canonical_candidate;

                if (evaluate_candidate(
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

            if (!evaluate_candidate(inst, std::move(trial), cost, canonical_trial))
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


bool shaw_removal_greedy_repair(
    const Instance &inst,
    Solution &sol,
    int remove_count)
{
    if (remove_count <= 0)
        return false;

    if (sol.truck_route.size() <= 1)
        return false;

    const long long old_cost = objective_function_impl(inst, sol);

    const std::vector<int> removed =
        choose_shaw_removal_set(inst, sol, remove_count);

    if (removed.empty())
        return false;

    std::vector<int> repair_pool;
    Solution partial =
        remove_customers_and_invalidated_flights(sol, removed, repair_pool);

    if (repair_pool.empty())
        return false;

    if (!greedy_repair(inst, partial, repair_pool))
        return false;

    long long new_cost = 0;
    Solution canonical_candidate;

    if (!evaluate_candidate(inst, std::move(partial), new_cost, canonical_candidate))
        return false;

    if (new_cost >= old_cost)
        return false;

    sol = std::move(canonical_candidate);
    return true;
}


bool shaw_removal_greedy_repair_random_small(
    const Instance &inst,
    Solution &sol)
{
    return shaw_removal_greedy_repair(inst, sol, 3);
}

bool shaw_removal_greedy_repair_random_medium(
    const Instance &inst,
    Solution &sol)
{
    return shaw_removal_greedy_repair(inst, sol, 5);
}

bool shaw_removal_greedy_repair_random_large(
    const Instance &inst,
    Solution &sol)
{
    return shaw_removal_greedy_repair(inst, sol, 8);
}