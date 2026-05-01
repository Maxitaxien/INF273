#include "operators/exchange_k.h"

#include "general/random.h"
#include "operators/customer_slot_helpers.h"
#include "operators/operator.h"
#include "operators/route_timing.h"
#include "solution_fixers/solution_fixers.h"
#include "tsp/linkernsolver.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"

#include <algorithm>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

namespace
{
constexpr int kExchangeKMediumRebuildAttempts = 1;
constexpr int kExchangeKLargeRounds = 20;

struct DroneRankedCandidate
{
    CustomerSlot slot;
    int customer = -1;
    long long downstream_delay_impact = 0;
    long long truck_wait = 0;
    long long drone_wait = 0;
    int land_idx = -1;
};

struct TruckRankedCandidate
{
    CustomerSlot slot;
    int customer = -1;
    long long weighted_incoming_cost = 0;
    long long incoming_edge_cost = 0;
    int truck_index = -1;
};

bool solutions_match(const Solution &lhs, const Solution &rhs)
{
    if (lhs.truck_route != rhs.truck_route || lhs.drones.size() != rhs.drones.size())
    {
        return false;
    }

    for (int drone = 0; drone < (int)(lhs.drones.size()); ++drone)
    {
        const DroneCollection &lhs_collection = lhs.drones[(size_t)drone];
        const DroneCollection &rhs_collection = rhs.drones[(size_t)drone];
        if (lhs_collection.launch_indices != rhs_collection.launch_indices ||
            lhs_collection.deliver_nodes != rhs_collection.deliver_nodes ||
            lhs_collection.land_indices != rhs_collection.land_indices)
        {
            return false;
        }
    }

    return true;
}

std::vector<int> collect_drone_customers(const Solution &solution)
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

void clear_all_drone_flights(Solution &solution)
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

void ensure_drone_collections(const Instance &instance, Solution &solution)
{
    if ((int)solution.drones.size() < instance.m)
    {
        solution.drones.resize((size_t)instance.m);
    }
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

std::vector<int> best_depot_path_from_customer_cycle(
    const Instance &instance,
    const std::vector<int> &customer_cycle,
    const std::vector<int> &reference_route)
{
    if (customer_cycle.empty())
    {
        return std::vector<int>{0};
    }

    std::vector<int> best_route;
    long long best_score = std::numeric_limits<long long>::max();
    int best_mismatch = std::numeric_limits<int>::max();

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

            const long long score = truck_arrival_sum_only(instance, route);
            const int mismatch = positional_mismatch(reference_route, route);
            if (score < best_score ||
                (score == best_score && mismatch < best_mismatch))
            {
                best_score = score;
                best_mismatch = mismatch;
                best_route = route;
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
    Solution &solution)
{
    if (solution.truck_route.empty() || solution.truck_route.front() != 0)
    {
        return false;
    }

    const std::vector<int> customer_nodes(
        solution.truck_route.begin() + 1,
        solution.truck_route.end());
    if (customer_nodes.size() < 4)
    {
        return true;
    }

    LinkernSolver solver(instance.truck_matrix);
    const LinkernTour optimized = solver.solve_route(customer_nodes);
    solution.truck_route =
        best_depot_path_from_customer_cycle(instance, optimized.tour, solution.truck_route);
    return true;
}

bool finalize_candidate_after_truck_rebuild(
    const Instance &instance,
    Solution &candidate_solution,
    long long *candidate_cost = nullptr)
{
    if (!rebuild_drone_customers_in_random_order(instance, candidate_solution))
    {
        return false;
    }

    simple_fix_validity(candidate_solution);
    if (!master_check(instance, candidate_solution, false))
    {
        return false;
    }

    if (candidate_cost != nullptr)
    {
        *candidate_cost = objective_function_impl(instance, candidate_solution);
    }

    return true;
}

void swap_deliveries(
    Solution &solution,
    const std::vector<CustomerSlot> &selected_slots)
{
    std::vector<unsigned char> selected_truck(solution.truck_route.size(), 0);
    std::vector<std::vector<unsigned char>> selected_drone(solution.drones.size());
    for (int drone = 0; drone < (int)solution.drones.size(); ++drone)
    {
        selected_drone[(size_t)drone].assign(
            solution.drones[(size_t)drone].deliver_nodes.size(),
            0);
    }

    std::vector<int> truck_to_drone;
    std::vector<int> drone_to_truck;

    for (const CustomerSlot &slot : selected_slots)
    {
        const int customer = read_customer_at_slot(solution, slot);
        if (slot.on_truck)
        {
            selected_truck[(size_t)slot.index] = 1;
            truck_to_drone.push_back(customer);
        }
        else
        {
            selected_drone[(size_t)slot.drone][(size_t)slot.index] = 1;
            drone_to_truck.push_back(customer);
        }
    }

    std::vector<int> new_truck_route;
    new_truck_route.reserve(
        solution.truck_route.size() - truck_to_drone.size() + drone_to_truck.size());
    new_truck_route.push_back(0);
    for (int idx = 1; idx < (int)solution.truck_route.size(); ++idx)
    {
        if (!selected_truck[(size_t)idx])
        {
            new_truck_route.push_back(solution.truck_route[(size_t)idx]);
        }
    }
    new_truck_route.insert(
        new_truck_route.end(),
        drone_to_truck.begin(),
        drone_to_truck.end());

    std::vector<DroneCollection> new_drones = solution.drones;
    for (int drone = 0; drone < (int)new_drones.size(); ++drone)
    {
        std::vector<int> kept_deliveries;
        kept_deliveries.reserve(new_drones[(size_t)drone].deliver_nodes.size());
        for (int idx = 0; idx < (int)new_drones[(size_t)drone].deliver_nodes.size(); ++idx)
        {
            if (!selected_drone[(size_t)drone][(size_t)idx])
            {
                kept_deliveries.push_back(
                    new_drones[(size_t)drone].deliver_nodes[(size_t)idx]);
            }
        }

        new_drones[(size_t)drone].deliver_nodes = std::move(kept_deliveries);
        new_drones[(size_t)drone].launch_indices.clear();
        new_drones[(size_t)drone].land_indices.clear();
    }

    if (!truck_to_drone.empty())
    {
        if (new_drones.empty())
        {
            new_drones.resize(1);
        }

        new_drones.front().deliver_nodes.insert(
            new_drones.front().deliver_nodes.end(),
            truck_to_drone.begin(),
            truck_to_drone.end());
    }

    solution.truck_route = std::move(new_truck_route);
    solution.drones = std::move(new_drones);
}

std::vector<CustomerSlot> slots_from_indices(
    const std::vector<CustomerSlot> &slots,
    const std::vector<int> &selected_indices)
{
    std::vector<CustomerSlot> selected_slots;
    selected_slots.reserve(selected_indices.size());
    for (const int idx : selected_indices)
    {
        selected_slots.push_back(slots[(size_t)idx]);
    }
    return selected_slots;
}

bool finalize_exchanged_solution(
    const Instance &instance,
    Solution &candidate_solution,
    long long *candidate_cost = nullptr)
{
    if (!rebuild_truck_route_with_linkern(instance, candidate_solution))
    {
        return false;
    }

    return finalize_candidate_after_truck_rebuild(
        instance,
        candidate_solution,
        candidate_cost);
}

bool evaluate_exchange_candidate_selection(
    const Instance &instance,
    const Solution &base_solution,
    const std::vector<CustomerSlot> &selected_slots,
    Solution &candidate_solution,
    long long &candidate_cost)
{
    if (selected_slots.empty())
    {
        return false;
    }

    candidate_solution = base_solution;
    ensure_drone_collections(instance, candidate_solution);
    swap_deliveries(candidate_solution, selected_slots);
    return finalize_exchanged_solution(
        instance,
        candidate_solution,
        &candidate_cost);
}

bool apply_random_exchange_k_swap_only(
    const Instance &instance,
    Solution &solution,
    ExchangeKSizes size)
{
    ensure_drone_collections(instance, solution);

    const std::vector<CustomerSlot> slots = collect_customer_slots(solution);
    const int slot_count = (int)slots.size();
    const int max_sample_size = std::min((int)size, slot_count);
    if (max_sample_size <= 0)
    {
        return false;
    }

    const int amount_swapped = rand_int(1, max_sample_size);
    const std::vector<int> selected_indices = sample_slot_indices(slot_count, amount_swapped);
    std::vector<CustomerSlot> selected_slots;
    selected_slots.reserve(selected_indices.size());
    for (const int idx : selected_indices)
    {
        selected_slots.push_back(slots[(size_t)idx]);
    }
    if (selected_slots.empty())
    {
        return false;
    }

    swap_deliveries(solution, selected_slots);
    return true;
}

std::vector<DroneRankedCandidate> rank_medium_drone_candidates(
    const Instance &instance,
    const Solution &solution)
{
    std::vector<DroneRankedCandidate> ranked;
    if (solution.truck_route.empty())
    {
        return ranked;
    }

    const RouteTiming timing = compute_route_timing(instance, solution);
    const int route_size = (int)solution.truck_route.size();

    for (int drone = 0; drone < (int)solution.drones.size(); ++drone)
    {
        const DroneCollection &collection = solution.drones[(size_t)drone];
        const int flight_count = std::min(
            std::min(
                (int)collection.launch_indices.size(),
                (int)collection.deliver_nodes.size()),
            (int)collection.land_indices.size());
        for (int flight_idx = 0; flight_idx < flight_count; ++flight_idx)
        {
            const int launch_idx = collection.launch_indices[(size_t)flight_idx];
            const int land_idx = collection.land_indices[(size_t)flight_idx];
            const int customer = collection.deliver_nodes[(size_t)flight_idx];
            if (launch_idx < 0 || land_idx <= launch_idx ||
                land_idx >= route_size || customer <= 0)
            {
                continue;
            }

            const int launch_node = solution.truck_route[(size_t)launch_idx];
            const int land_node = solution.truck_route[(size_t)land_idx];
            const long long launch_time = std::max(
                timing.truck_arrival[(size_t)launch_idx],
                timing.drone_ready_at_stop[(size_t)drone][(size_t)launch_idx]);
            const long long out_time = instance.drone_matrix[(size_t)launch_node][(size_t)customer];
            const long long back_time = instance.drone_matrix[(size_t)customer][(size_t)land_node];
            const long long drone_return = launch_time + out_time + back_time;
            const long long truck_wait = std::max(
                drone_return - timing.truck_arrival[(size_t)land_idx],
                0LL);
            const long long drone_wait = std::max(
                timing.truck_arrival[(size_t)land_idx] - drone_return,
                0LL);
            const long long downstream_stops = std::max(0, route_size - land_idx - 1);

            ranked.push_back(DroneRankedCandidate{
                CustomerSlot{false, flight_idx, drone},
                customer,
                truck_wait * downstream_stops,
                truck_wait,
                drone_wait,
                land_idx,
            });
        }
    }

    std::sort(
        ranked.begin(),
        ranked.end(),
        [](const DroneRankedCandidate &lhs, const DroneRankedCandidate &rhs) {
            return std::tie(
                       lhs.downstream_delay_impact,
                       lhs.truck_wait,
                       lhs.drone_wait,
                       lhs.land_idx) >
                std::tie(
                       rhs.downstream_delay_impact,
                       rhs.truck_wait,
                       rhs.drone_wait,
                       rhs.land_idx);
        });
    return ranked;
}

std::vector<TruckRankedCandidate> rank_medium_truck_candidates(
    const Instance &instance,
    const Solution &solution)
{
    std::vector<TruckRankedCandidate> ranked;
    const int route_size = (int)solution.truck_route.size();
    for (int idx = 1; idx < route_size; ++idx)
    {
        const int prev = solution.truck_route[(size_t)(idx - 1)];
        const int curr = solution.truck_route[(size_t)idx];
        const long long incoming_edge_cost =
            instance.truck_matrix[(size_t)prev][(size_t)curr];
        const long long downstream_customer_count = route_size - idx;
        ranked.push_back(TruckRankedCandidate{
            CustomerSlot{true, idx, -1},
            curr,
            incoming_edge_cost * downstream_customer_count,
            incoming_edge_cost,
            idx,
        });
    }

    std::sort(
        ranked.begin(),
        ranked.end(),
        [](const TruckRankedCandidate &lhs, const TruckRankedCandidate &rhs) {
            return std::tie(
                       lhs.weighted_incoming_cost,
                       lhs.incoming_edge_cost,
                       lhs.truck_index) >
                std::tie(
                       rhs.weighted_incoming_cost,
                       rhs.incoming_edge_cost,
                       rhs.truck_index);
        });
    return ranked;
}

std::vector<CustomerSlot> build_exchange_k_medium_candidate_pool(
    const Instance &instance,
    const Solution &solution)
{
    const std::vector<DroneRankedCandidate> ranked_drone =
        rank_medium_drone_candidates(instance, solution);
    const std::vector<TruckRankedCandidate> ranked_truck =
        rank_medium_truck_candidates(instance, solution);

    std::vector<CustomerSlot> pool;
    pool.reserve(10);
    std::vector<unsigned char> seen_customer((size_t)std::max(0, instance.n + 1), 0);

    const int drone_take = std::min(5, (int)ranked_drone.size());
    for (int idx = 0; idx < drone_take; ++idx)
    {
        const DroneRankedCandidate &candidate = ranked_drone[(size_t)idx];
        if (candidate.customer <= 0 ||
            candidate.customer >= (int)seen_customer.size() ||
            seen_customer[(size_t)candidate.customer])
        {
            continue;
        }

        seen_customer[(size_t)candidate.customer] = 1;
        pool.push_back(candidate.slot);
    }

    const int truck_take = std::min(5, (int)ranked_truck.size());
    for (int idx = 0; idx < truck_take; ++idx)
    {
        const TruckRankedCandidate &candidate = ranked_truck[(size_t)idx];
        if (candidate.customer <= 0 ||
            candidate.customer >= (int)seen_customer.size() ||
            seen_customer[(size_t)candidate.customer])
        {
            continue;
        }

        seen_customer[(size_t)candidate.customer] = 1;
        pool.push_back(candidate.slot);
    }

    return pool;
}

bool is_balanced_medium_selection(const std::vector<CustomerSlot> &selected_slots)
{
    if (selected_slots.size() != 4)
    {
        return false;
    }

    int truck_count = 0;
    for (const CustomerSlot &slot : selected_slots)
    {
        truck_count += slot.on_truck ? 1 : 0;
    }

    return truck_count == 2;
}

bool exchange_k_impl(
    const Instance &instance,
    Solution &solution,
    ExchangeKSizes size)
{
    const std::vector<CustomerSlot> slots = collect_customer_slots(solution);
    const int slot_count = (int)slots.size();
    const int max_sample_size = std::min((int)size, slot_count);
    if (max_sample_size <= 0)
    {
        return false;
    }

    const int amount_swapped = rand_int(1, max_sample_size);
    const std::vector<int> selected_indices = sample_slot_indices(slot_count, amount_swapped);
    Solution candidate;
    long long candidate_cost = 0;
    if (!evaluate_exchange_candidate_selection(
            instance,
            solution,
            slots_from_indices(slots, selected_indices),
            candidate,
            candidate_cost))
    {
        return false;
    }

    if (solutions_match(candidate, solution))
    {
        return false;
    }

    drone_rendezvous_shift_best_improvement(instance, candidate);
    solution = std::move(candidate);
    return true;
}
} // namespace

bool exchange_k_small(const Instance &instance, Solution &solution)
{
    return exchange_k_impl(instance, solution, ExchangeKSizes::S);
}

bool exchange_k_medium(const Instance &instance, Solution &solution)
{
    const long long base_cost = objective_function_impl(instance, solution);
    const std::vector<CustomerSlot> pool =
        build_exchange_k_medium_candidate_pool(instance, solution);
    if (pool.size() < 4)
    {
        return false;
    }

    bool found_improvement = false;
    long long best_cost = base_cost;
    Solution best_candidate;

    for (size_t i = 0; i < pool.size(); ++i)
    {
        for (size_t j = i + 1; j < pool.size(); ++j)
        {
            for (size_t k = j + 1; k < pool.size(); ++k)
            {
                for (size_t l = k + 1; l < pool.size(); ++l)
                {
                    const std::vector<CustomerSlot> selected = {
                        pool[i],
                        pool[j],
                        pool[k],
                        pool[l],
                    };
                    if (!is_balanced_medium_selection(selected))
                    {
                        continue;
                    }

                    bool found_subset_candidate = false;
                    long long subset_best_cost = std::numeric_limits<long long>::max();
                    Solution subset_best_candidate;

                    for (int attempt = 0; attempt < kExchangeKMediumRebuildAttempts; ++attempt)
                    {
                        Solution candidate;
                        long long candidate_cost = 0;
                        if (!evaluate_exchange_candidate_selection(
                                instance,
                                solution,
                                selected,
                                candidate,
                                candidate_cost) ||
                            solutions_match(candidate, solution))
                        {
                            continue;
                        }

                        if (!found_subset_candidate || candidate_cost < subset_best_cost)
                        {
                            found_subset_candidate = true;
                            subset_best_cost = candidate_cost;
                            subset_best_candidate = std::move(candidate);
                        }
                    }

                    if (!found_subset_candidate || subset_best_cost >= best_cost)
                    {
                        continue;
                    }

                    found_improvement = true;
                    best_cost = subset_best_cost;
                    best_candidate = std::move(subset_best_candidate);
                }
            }
        }
    }

    if (!found_improvement)
    {
        return false;
    }

    solution = std::move(best_candidate);
    return true;
}

bool exchange_k_large(const Instance &instance, Solution &solution)
{
    Solution working = solution;
    bool found_raw_change = false;

    for (int round = 0; round < kExchangeKLargeRounds; ++round)
    {
        if (!apply_random_exchange_k_swap_only(instance, working, ExchangeKSizes::L))
        {
            continue;
        }

        found_raw_change = true;
    }

    if (!found_raw_change)
    {
        return false;
    }

    long long ignored_cost = 0;
    if (!finalize_exchanged_solution(instance, working, &ignored_cost))
    {
        return false;
    }

    drone_rendezvous_shift_best_improvement(instance, working);
    if (solutions_match(working, solution))
    {
        return false;
    }

    solution = std::move(working);
    return true;
}
