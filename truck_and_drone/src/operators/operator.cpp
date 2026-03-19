#include "operators/operator.h"
#include "operators/nearest_neighbour_reassign.h"
#include "operators/one_reinsert.h"
#include "operators/replace_truck_delivery.h"
#include "operators/two_opt.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <iostream>
#include <random>
#include <utility>
#include <vector>

extern std::mt19937 gen; // reuse global generator
const long long INF = 4e18;

Operator make_alns_operator(const ALNSOperator &op)
{
    return [op](const Instance &instance, Solution &sol) {
        Solution candidate = sol;

        if (!op.removal || !op.insertion)
        {
            return false;
        }

        if (!op.removal(instance, candidate, op.neighbourhood_size))
        {
            return false;
        }

        if (!op.insertion(instance, candidate, op.neighbourhood_size, op.regret_k))
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
                if (pop > 1 && (pop - 2 >= sol.drones.size() || sol.drones[pop - 2].deliver_nodes.empty()))
                    continue;
                if (insert > 1 && (insert - 2 >= sol.drones.size()))
                    continue;

                Solution neighbor = sol; // one copy per trial
                if (one_reinsert(instance, neighbor, pop, insert, idx))
                {
                    if (master_check(instance, neighbor, false))
                    {
                        neighbors.push_back(neighbor);
                    }
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
        (pop > 1 && (pop - 2 >= sol.drones.size() || sol.drones[pop - 2].deliver_nodes.empty())) ||
        (insert > 1 && insert - 2 >= sol.drones.size()));

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
    return false;
}

bool replace_truck_delivery_greedy(const Instance &instance, Solution &sol)
{
    bool success = false;
    long long best_cost = INF;
    int best_i = -1, best_drone = -1;

    for (int drone = 0; drone < (int)sol.drones.size(); ++drone)
    {
        for (int i = 1; i < (int)sol.truck_route.size(); ++i)
        {
            Solution copy = sol; // fresh copy each time
            if (replace_truck_delivery(instance, copy, i, drone) &&
                master_check(instance, copy, false)) // check the candidate
            {
                long long cost = objective_function_impl(instance, copy);
                if (cost < best_cost)
                {
                    best_cost = cost;
                    best_i = i;
                    best_drone = drone;
                    success = true;
                }
            }
        }
    }

    if (success)
    {
        replace_truck_delivery(instance, sol, best_i, best_drone);
    }
    return success;
}

bool two_opt_random(const Instance &inst, Solution &sol)
{
    // Random i, j, different from each other, each in range 1 -> sol.truck_route.size() - 1
    if (sol.truck_route.size() < 4)
        return false; // Need at least 4 nodes for meaningful 2-opt

    std::uniform_int_distribution<int> dist(1, sol.truck_route.size() - 1);
    int i = dist(gen);
    int j = dist(gen);
    while (i == j)
    {
        j = dist(gen);
    }

    two_opt(inst, sol, i, j);
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

    nearest_neighbour_reassign(inst, sol, i);
    return true;
}
