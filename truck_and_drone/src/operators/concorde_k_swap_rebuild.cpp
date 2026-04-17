#include "operators/operator.h"

#include "general/random.h"
#include "operators/customer_slot_helpers.h"
#include "solution_fixers/solution_fixers.h"
#include "tsp/linkernsolver.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

namespace
{
long long truck_open_length(
    const Instance &instance,
    const std::vector<int> &route)
{
    long long total = 0;
    for (size_t idx = 1; idx < route.size(); ++idx)
    {
        total += instance.truck_matrix[(size_t)route[idx - 1]][(size_t)route[idx]];
    }

    return total;
}

long long truck_arrival_sum_only(
    const Instance &instance,
    const std::vector<int> &route)
{
    long long total_arrival = 0;
    long long arrival = 0;
    for (size_t idx = 1; idx < route.size(); ++idx)
    {
        arrival += instance.truck_matrix[(size_t)route[idx - 1]][(size_t)route[idx]];
        total_arrival += arrival;
    }

    return total_arrival;
}

int positional_mismatch(
    const std::vector<int> &reference_route,
    const std::vector<int> &candidate_route)
{
    const size_t limit = std::min(reference_route.size(), candidate_route.size());
    int mismatch = 0;
    for (size_t idx = 0; idx < limit; ++idx)
    {
        if (reference_route[idx] != candidate_route[idx])
        {
            ++mismatch;
        }
    }

    mismatch += static_cast<int>(
        std::max(reference_route.size(), candidate_route.size()) - limit);
    return mismatch;
}

void ensure_drone_collections(
    const Instance &instance,
    Solution &solution)
{
    if ((int)(solution.drones.size()) < instance.m)
    {
        solution.drones.resize((size_t)instance.m);
    }
}

bool apply_random_k_slot_permutation(
    Solution &solution,
    int k)
{
    const std::vector<CustomerSlot> slots = collect_customer_slots(solution);
    const int slot_count = static_cast<int>(slots.size());
    const int sample_size = std::min(k, slot_count);
    if (sample_size < 2)
    {
        return false;
    }

    const std::vector<int> selected_indices =
        sample_slot_indices(slot_count, sample_size);

    std::vector<int> customers;
    customers.reserve((size_t)sample_size);
    for (const int selected_idx : selected_indices)
    {
        customers.push_back(read_customer_at_slot(solution, slots[(size_t)selected_idx]));
    }

    std::vector<int> permuted = customers;
    for (int attempt = 0; attempt < 8; ++attempt)
    {
        random_shuffle(permuted);
        if (permuted != customers)
        {
            break;
        }
    }

    if (permuted == customers)
    {
        return false;
    }

    for (int i = 0; i < sample_size; ++i)
    {
        write_customer_at_slot(
            solution,
            slots[(size_t)selected_indices[(size_t)i]],
            permuted[(size_t)i]);
    }

    return true;
}

std::vector<int> choose_best_depot_path_from_cycle(
    const Instance &instance,
    const std::vector<int> &customer_cycle,
    const std::vector<int> &reference_route,
    bool score_by_arrival)
{
    if (customer_cycle.empty())
    {
        return std::vector<int>{0};
    }

    std::vector<int> best_route;
    long long best_score = std::numeric_limits<long long>::max();
    int best_mismatch = std::numeric_limits<int>::max();

    const auto route_score = [&](const std::vector<int> &route) {
        return score_by_arrival
            ? truck_arrival_sum_only(instance, route)
            : truck_open_length(instance, route);
    };

    const auto consider_order = [&](const std::vector<int> &order) {
        const int customer_count = static_cast<int>(order.size());
        for (int cut = 0; cut < customer_count; ++cut)
        {
            std::vector<int> route;
            route.reserve((size_t)customer_count + 1);
            route.push_back(0);
            for (int offset = 0; offset < customer_count; ++offset)
            {
                route.push_back(order[(size_t)((cut + offset) % customer_count)]);
            }

            const long long score = route_score(route);
            const int mismatch = positional_mismatch(reference_route, route);
            if (score < best_score ||
                (score == best_score && mismatch < best_mismatch))
            {
                best_score = score;
                best_mismatch = mismatch;
                best_route = std::move(route);
            }
        }
    };

    consider_order(customer_cycle);

    std::vector<int> reversed_cycle = customer_cycle;
    std::reverse(reversed_cycle.begin(), reversed_cycle.end());
    consider_order(reversed_cycle);

    return best_route;
}

bool rebuild_truck_route_with_linkern(
    const Instance &instance,
    Solution &solution,
    const ConcordeKSwapRebuildConfig &config)
{
    if ((int)(solution.truck_route.size()) < 3 || solution.truck_route.front() != 0)
    {
        return false;
    }

    const std::vector<int> customer_nodes(
        solution.truck_route.begin() + 1,
        solution.truck_route.end());
    if (customer_nodes.size() < 2)
    {
        return true;
    }

    LinkernSolver solver(instance.truck_matrix);
    const LinkernTour optimized = solver.solve_route(
        customer_nodes,
        -1,
        config.stallcount,
        config.repeatcount,
        config.kicktype,
        config.time_bound);

    solution.truck_route = choose_best_depot_path_from_cycle(
        instance,
        optimized.tour,
        solution.truck_route,
        config.score_depot_cut_by_arrival);
    return true;
}

std::vector<int> collect_drone_customers(
    const Solution &solution)
{
    std::vector<int> drone_customers;
    for (const DroneCollection &drone : solution.drones)
    {
        drone_customers.insert(
            drone_customers.end(),
            drone.deliver_nodes.begin(),
            drone.deliver_nodes.end());
    }
    return drone_customers;
}

void clear_all_drone_flights(
    Solution &solution)
{
    for (DroneCollection &drone : solution.drones)
    {
        drone.launch_indices.clear();
        drone.land_indices.clear();
        drone.deliver_nodes.clear();
    }
}

bool assign_single_drone_customer_best_local(
    const Instance &instance,
    Solution &partial_solution,
    int customer)
{
    bool found = false;
    long long best_score = std::numeric_limits<long long>::max();
    Solution best_candidate;

    for (int drone = 0; drone < (int)(partial_solution.drones.size()); ++drone)
    {
        Solution candidate = partial_solution;
        auto [assigned_ok, ignored_solution] = greedy_assign_launch_and_land_assume_valid(
            instance,
            candidate,
            customer,
            drone);
        (void)ignored_solution;
        if (!assigned_ok)
        {
            continue;
        }

        // TODO: replace this partial-solution score with a dedicated per-customer
        // assignment scorer if you want tighter control over the rebuild order.
        const long long score = objective_function_impl(instance, candidate);
        if (!found || score < best_score)
        {
            found = true;
            best_score = score;
            best_candidate = std::move(candidate);
        }
    }

    if (!found)
    {
        return false;
    }

    partial_solution = std::move(best_candidate);
    return true;
}

bool rebuild_drone_customers_in_random_order(
    const Instance &instance,
    Solution &solution)
{
    std::vector<int> drone_customers = collect_drone_customers(solution);
    if (drone_customers.empty())
    {
        return true;
    }

    random_shuffle(drone_customers);
    clear_all_drone_flights(solution);

    for (const int customer : drone_customers)
    {
        if (!assign_single_drone_customer_best_local(instance, solution, customer))
        {
            return false;
        }
    }

    return true;
}
}

bool concorde_k_swap_rebuild(
    const Instance &instance,
    Solution &sol,
    const ConcordeKSwapRebuildConfig &config)
{
    if (config.k < 2)
    {
        return false;
    }

    const long long current_cost = objective_function_impl(instance, sol);

    Solution candidate = sol;
    ensure_drone_collections(instance, candidate);

    if (!apply_random_k_slot_permutation(candidate, config.k))
    {
        return false;
    }

    // Step 1: perturb the combined representation.
    // Step 2: use linkern as a fast truck reorder oracle on the new truck set.
    if (!rebuild_truck_route_with_linkern(instance, candidate, config))
    {
        return false;
    }

    // Step 3: rebuild the drone schedule from scratch in randomized customer order.
    // TODO: this currently uses greedy per-customer local assignment; replace with
    // a stronger exact or bounded-best assignment rule when you are ready.
    if (!rebuild_drone_customers_in_random_order(instance, candidate))
    {
        return false;
    }

    simple_fix_validity(candidate);
    if (!master_check(instance, candidate, false))
    {
        candidate = fix_overall_feasibility(instance, candidate);
        if (!master_check(instance, candidate, false))
        {
            return false;
        }
    }

    const long long candidate_cost = objective_function_impl(instance, candidate);
    if (config.require_improvement && candidate_cost >= current_cost)
    {
        return false;
    }

    sol = std::move(candidate);
    return true;
}

bool concorde_k_swap_rebuild(
    const Instance &instance,
    Solution &sol)
{
    return concorde_k_swap_rebuild(
        instance,
        sol,
        ConcordeKSwapRebuildConfig{});
}
