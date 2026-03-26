#include "operators/operator.h"
#include "operators/drone_planner.h"
#include "operators/nearest_neighbour_reassign.h"
#include "operators/one_reinsert.h"
#include "operators/replace_drone_delivery.h"
#include "operators/replace_truck_delivery.h"
#include "operators/solution_fixers.h"
#include "operators/three_opt.h"
#include "operators/two_opt.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <algorithm>
#include <array>
#include <numeric>
#include <set>
#include <random>
#include <utility>
#include <vector>

extern std::mt19937 gen; // reuse global generator
const long long INF = 4e18;

namespace
{
struct CustomerSlot
{
    bool on_truck = false;
    int index = -1;
    int drone = -1;
};

int count_drone_deliveries(const Solution &sol)
{
    int delivery_count = 0;
    for (const DroneCollection &drone : sol.drones)
    {
        delivery_count += (int)(drone.deliver_nodes.size());
    }

    return delivery_count;
}

bool planner_improve_with_budget(
    const Instance &instance,
    Solution &sol,
    int iterations,
    int max_flights_per_customer,
    int drone_to_replan)
{
    if (count_drone_deliveries(sol) == 0)
    {
        return false;
    }

    const long long current_cost = objective_function_impl(instance, sol);
    const auto [planned_cost, planned_solution] = drone_planner(
        instance,
        sol,
        iterations,
        max_flights_per_customer,
        drone_to_replan);

    if (planned_cost >= current_cost)
    {
        return false;
    }

    sol = planned_solution;
    return true;
}

std::vector<CustomerSlot> collect_customer_slots(const Solution &sol)
{
    std::vector<CustomerSlot> slots;
    int total_slots = std::max(0, (int)(sol.truck_route.size()) - 1);
    for (const DroneCollection &drone : sol.drones)
    {
        total_slots += (int)(drone.deliver_nodes.size());
    }

    slots.reserve(total_slots);
    for (int idx = 1; idx < (int)(sol.truck_route.size()); ++idx)
    {
        slots.push_back(CustomerSlot{true, idx, -1});
    }

    for (int drone = 0; drone < (int)(sol.drones.size()); ++drone)
    {
        for (int idx = 0; idx < (int)(sol.drones[drone].deliver_nodes.size()); ++idx)
        {
            slots.push_back(CustomerSlot{false, idx, drone});
        }
    }

    return slots;
}

int read_customer_at_slot(const Solution &sol, const CustomerSlot &slot)
{
    if (slot.on_truck)
    {
        return sol.truck_route[slot.index];
    }

    return sol.drones[slot.drone].deliver_nodes[slot.index];
}

void write_customer_at_slot(Solution &sol, const CustomerSlot &slot, int customer)
{
    if (slot.on_truck)
    {
        sol.truck_route[slot.index] = customer;
        return;
    }

    sol.drones[slot.drone].deliver_nodes[slot.index] = customer;
}

std::vector<int> sample_slot_indices(int slot_count, int sample_size)
{
    std::vector<int> indices(slot_count);
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), gen);
    indices.resize(sample_size);
    return indices;
}

bool repair_with_drone_planner_if_needed(
    const Instance &inst,
    Solution &candidate)
{
    if (master_check(inst, candidate, false))
    {
        return true;
    }

    const auto [planned_cost, planned_solution] = drone_planner(inst, candidate);
    (void)planned_cost;
    if (!master_check(inst, planned_solution, false))
    {
        return false;
    }

    candidate = planned_solution;
    return true;
}
}

Operator make_alns_operator(const ALNSOperator &op)
{
    return [op](const Instance &instance, Solution &sol) {
        Solution candidate = sol;

        if (!op.removal || !op.insertion)
        {
            return false;
        }

        const auto [removed_ok, removed_nodes] =
            op.removal(instance, candidate, op.neighbourhood_size);
        if (!removed_ok)
        {
            return false;
        }

        if (!op.insertion(instance, candidate, removed_nodes, op.regret_k))
        {
            return false;
        }

        sol = std::move(candidate);
        return true;
    };
}

NamedOperator make_named_alns_operator(const NamedALNSOperator &op)
{
    return NamedOperator{op.name, make_alns_operator(op.op)};
}

std::vector<NamedOperator> combine_alns_operator_pairs(
    const std::vector<NamedRemovalHeuristic> &removals,
    const std::vector<NamedInsertionHeuristic> &insertions,
    int neighbourhood_size,
    int regret_k)
{
    std::vector<NamedOperator> combined;
    combined.reserve(removals.size() * insertions.size());

    for (const NamedRemovalHeuristic &removal : removals)
    {
        for (const NamedInsertionHeuristic &insertion : insertions)
        {
            combined.push_back(make_named_alns_operator({
                removal.name + " + " + insertion.name,
                ALNSOperator{removal.op, insertion.op, neighbourhood_size, regret_k},
            }));
        }
    }

    return combined;
}

// Generate all valid neighbors (optional, may be expensive)
std::vector<Solution> one_reinsert_operator(const Instance &instance, const Solution &sol)
{
    std::vector<Solution> neighbors;

    for (int pop = 1; pop <= 3; pop++)
    {
        for (int insert = 1; insert <= 3; insert++)
        {
            int truck_size_after_pop = sol.truck_route.size();
            if (pop == 1 && !sol.truck_route.empty())
                truck_size_after_pop--;

            for (int idx = 1; idx <= truck_size_after_pop; ++idx)
            {
                // Skip invalid drone pop/insert
                if (pop > 1 && (pop - 2 >= (int)(sol.drones.size()) || sol.drones[pop - 2].deliver_nodes.empty()))
                    continue;
                if (insert > 1 && (insert - 2 >= (int)(sol.drones.size())))
                    continue;

                Solution neighbor = sol; // one copy per trial
                if (one_reinsert(instance, neighbor, pop, insert, idx))
                {
                    neighbors.push_back(neighbor);
                }
            }
        }
    }

    return neighbors;
}

// Generate a single random neighbor
bool one_reinsert_random(const Instance &instance, Solution &sol)
{
    std::uniform_int_distribution<int> pop_dist(1, 3);
    std::uniform_int_distribution<int> insert_dist(1, 3);

    int pop, insert;

    do
    {
        pop = pop_dist(gen);
        insert = insert_dist(gen);
    } while (
        (pop > 1 && (pop - 2 >= (int)(sol.drones.size()) || sol.drones[pop - 2].deliver_nodes.empty())) ||
        (insert > 1 && insert - 2 >= (int)(sol.drones.size())));

    int truck_size_after_pop = sol.truck_route.size();
    if (pop == 1 && !sol.truck_route.empty())
        truck_size_after_pop--;

    if (truck_size_after_pop <= 1)
        return false; // cannot pop/insert

    std::uniform_int_distribution<int> idx_dist(1, truck_size_after_pop);
    int idx = idx_dist(gen);

    return one_reinsert(instance, sol, pop, insert, idx); // in-place
}

// Generate a one_reinsert candidate by greedily evaluating insertion candidates
bool one_reinsert_greedy(const Instance &instance, Solution &sol)
{
    (void)instance;
    (void)sol;
    return false;
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
    long long best_cost = INF;
    Solution best_solution;
    Solution candidate = sol;

    for (int drone = 0; drone < (int)sol.drones.size(); ++drone)
    {
        for (int i = 1; i < (int)sol.truck_route.size(); ++i)
        {
            candidate = sol;
            if (replace_truck_delivery(instance, candidate, i, drone))
            {
                long long cost = objective_function_impl(instance, candidate);
                if (cost < best_cost)
                {
                    best_cost = cost;
                    best_solution = std::move(candidate);
                    success = true;
                }
            }
        }
    }

    if (success)
    {
        sol = std::move(best_solution);
    }
    return success;
}

bool replace_drone_delivery_random(const Instance &instance, Solution &sol)
{
    std::vector<std::pair<int, int>> candidates;
    for (int drone = 0; drone < (int)(sol.drones.size()); ++drone)
    {
        const int flight_count = (int)(sol.drones[drone].deliver_nodes.size());
        for (int flight_idx = 0; flight_idx < flight_count; ++flight_idx)
        {
            candidates.emplace_back(drone, flight_idx);
        }
    }

    if (candidates.empty())
    {
        return false;
    }

    std::shuffle(candidates.begin(), candidates.end(), gen);
    const int max_attempts = std::min(8, (int)(candidates.size()));
    Solution candidate = sol;

    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
        const auto [drone, flight_idx] = candidates[attempt];
        candidate = sol;
        if (replace_drone_delivery(instance, candidate, drone, flight_idx))
        {
            sol = std::move(candidate);
            return true;
        }
    }

    return false;
}

bool replace_drone_delivery_greedy(const Instance &instance, Solution &sol)
{
    bool success = false;
    long long best_cost = INF;
    Solution best_solution;
    Solution candidate = sol;

    for (int drone = 0; drone < (int)(sol.drones.size()); ++drone)
    {
        const int flight_count = (int)(sol.drones[drone].deliver_nodes.size());
        for (int flight_idx = 0; flight_idx < flight_count; ++flight_idx)
        {
            candidate = sol;
            if (!replace_drone_delivery(instance, candidate, drone, flight_idx))
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

bool drone_demotion_shake(const Instance &instance, Solution &sol)
{
    const int delivery_count = count_drone_deliveries(sol);
    if (delivery_count == 0)
    {
        return false;
    }

    Solution candidate = sol;
    const int demotions_to_apply = delivery_count >= 20 ? 2 : 1;
    bool changed = false;

    for (int i = 0; i < demotions_to_apply; ++i)
    {
        if (!replace_drone_delivery_greedy(instance, candidate))
        {
            break;
        }

        changed = true;
    }

    if (!changed)
    {
        return false;
    }

    sol = std::move(candidate);
    return true;
}

bool two_opt_random(const Instance &inst, Solution &sol)
{
    const std::vector<CustomerSlot> slots = collect_customer_slots(sol);
    if ((int)(slots.size()) < 2)
    {
        return false;
    }

    const std::vector<int> selected = sample_slot_indices((int)(slots.size()), 2);
    Solution candidate = sol;
    const int first_customer = read_customer_at_slot(sol, slots[selected[0]]);
    const int second_customer = read_customer_at_slot(sol, slots[selected[1]]);

    write_customer_at_slot(candidate, slots[selected[0]], second_customer);
    write_customer_at_slot(candidate, slots[selected[1]], first_customer);

    if (!repair_with_drone_planner_if_needed(inst, candidate))
    {
        return false;
    }

    sol = std::move(candidate);
    return true;
}

bool three_opt_random(const Instance &inst, Solution &sol)
{
    const std::vector<CustomerSlot> slots = collect_customer_slots(sol);
    if ((int)(slots.size()) < 3)
    {
        return false;
    }

    const long long current_truck_cost =
        objective_function_truck_only(inst, sol.truck_route);
    const std::vector<int> selected = sample_slot_indices((int)(slots.size()), 3);
    const std::array<int, 3> customers = {
        read_customer_at_slot(sol, slots[selected[0]]),
        read_customer_at_slot(sol, slots[selected[1]]),
        read_customer_at_slot(sol, slots[selected[2]]),
    };
    constexpr std::array<std::array<int, 3>, 3> permutations = {{
        {{0, 2, 1}},
        {{1, 0, 2}},
        {{2, 1, 0}},
        // {{1, 2, 0}}, // 3-cycle, currently disabled as a weaker truck surrogate candidate.
        // {{2, 0, 1}}, // 3-cycle, currently disabled as a weaker truck surrogate candidate.
    }};
    std::array<int, 3> permutation_order = {0, 1, 2};
    std::shuffle(permutation_order.begin(), permutation_order.end(), gen);
    const int permutation_budget = std::min(
        (int)(permutation_order.size()),
        inst.n >= 50 ? 2 : 3);

    bool found_improvement = false;
    long long best_truck_cost = current_truck_cost;
    Solution best_solution;
    Solution candidate = sol;

    for (int attempt = 0; attempt < permutation_budget; ++attempt)
    {
        candidate = sol;
        const auto &permutation = permutations[permutation_order[attempt]];
        for (int idx = 0; idx < 3; ++idx)
        {
            write_customer_at_slot(
                candidate,
                slots[selected[idx]],
                customers[permutation[idx]]);
        }

        if (!repair_with_drone_planner_if_needed(inst, candidate))
        {
            continue;
        }

        const long long candidate_truck_cost =
            objective_function_truck_only(inst, candidate.truck_route);
        if (candidate_truck_cost < best_truck_cost)
        {
            best_truck_cost = candidate_truck_cost;
            best_solution = std::move(candidate);
            found_improvement = true;
        }
    }

    if (!found_improvement)
    {
        return false;
    }

    sol = std::move(best_solution);
    return true;
}

bool nearest_neighbour_reassign_random(const Instance &inst, Solution &sol)
{
    // The route has depot only at the start; the final entry is a customer.
    // We must never select the last customer index because nearest_neighbour_reassign
    // uses `i+1` and would overflow in that case.
    if (sol.truck_route.size() <= 2)
        return false;

    int max_valid_idx = (int)sol.truck_route.size() - 2; // exclude last customer index
    std::uniform_int_distribution<int> dist(1, max_valid_idx);
    int i = dist(gen);

    return nearest_neighbour_reassign(inst, sol, i);
}

bool drone_planner_improve(const Instance &instance, Solution &sol)
{
    if (instance.n >= 50)
    {
        return drone_planner_light_improve(instance, sol);
    }

    return planner_improve_with_budget(instance, sol, 5, 0, -1);
}

bool drone_planner_light_improve(const Instance &instance, Solution &sol)
{
    std::vector<int> candidate_drones;
    candidate_drones.reserve(sol.drones.size());
    int drone_deliveries = 0;
    for (int drone = 0; drone < (int)(sol.drones.size()); ++drone)
    {
        const int deliveries = (int)(sol.drones[drone].deliver_nodes.size());
        drone_deliveries += deliveries;
        if (deliveries > 0)
        {
            candidate_drones.push_back(drone);
        }
    }

    if (candidate_drones.empty())
    {
        return false;
    }

    std::uniform_int_distribution<int> drone_dist(0, (int)(candidate_drones.size()) - 1);
    const int drone_to_replan = candidate_drones[drone_dist(gen)];
    const int iterations = drone_deliveries >= 20 ? 1 : 2;
    const int max_flights_per_customer = drone_deliveries >= 20 ? 8 : 12;
    return planner_improve_with_budget(
        instance,
        sol,
        iterations,
        max_flights_per_customer,
        drone_to_replan);
}
