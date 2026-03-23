#include "operators/alns/greedy_insert.h"
#include "general/roulette_wheel_selection.h"
#include "operators/drone_planner.h"
#include "operators/solution_fixers.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <algorithm>
#include <limits>
#include <queue>
#include <utility>

namespace
{
bool has_any_drone_deliveries(const Solution &sol)
{
    for (const DroneCollection &drone : sol.drones)
    {
        if (!drone.deliver_nodes.empty())
        {
            return true;
        }
    }

    return false;
}

Candidate generate_new_candidate(
    const Instance& inst,
    const Solution& sol,
    const std::vector<int> &to_insert,
    const std::vector<int> &insert_positions)
{
    Solution candidate_sol = sol;
    const int truck_insert_slots = sol.truck_route.size();
    bool inserted_into_drone = false;

    for (int i = (int)(insert_positions.size()) - 1; i >= 0; --i)
    {
        const int encoded_position = insert_positions[i];
        if (encoded_position < truck_insert_slots)
        {
            const int truck_insert_position = encoded_position + 1;
            candidate_sol.truck_route.insert(
                candidate_sol.truck_route.begin() + truck_insert_position,
                to_insert[i]
            );
        }
    }

    for (int i = 0; i < (int)(insert_positions.size()); ++i)
    {
        const int encoded_position = insert_positions[i];
        if (encoded_position >= truck_insert_slots)
        {
            const int drone = encoded_position - truck_insert_slots;
            candidate_sol.drones[drone].deliver_nodes.push_back(to_insert[i]);
            inserted_into_drone = true;
        }
    }

    if (!inserted_into_drone && !has_any_drone_deliveries(sol))
    {
        return Candidate{candidate_sol, objective_function_impl(inst, candidate_sol)};
    }

    const auto [score, planned_sol] = drone_planner(inst, candidate_sol);
    return Candidate{planned_sol, score};
}

bool evaluate_single_insert_candidate(
    const Instance &inst,
    const Solution &sol,
    int delivery,
    int encoded_position,
    bool base_has_drone_deliveries,
    Candidate &candidate)
{
    Solution candidate_sol = sol;
    const int truck_insert_slots = sol.truck_route.size();

    if (encoded_position < truck_insert_slots)
    {
        const int truck_insert_position = encoded_position + 1;
        candidate_sol.truck_route.insert(
            candidate_sol.truck_route.begin() + truck_insert_position,
            delivery);

        if (!base_has_drone_deliveries)
        {
            const long long score = objective_function_impl(inst, candidate_sol);
            candidate = Candidate{std::move(candidate_sol), score};
            return true;
        }

        const auto [score, planned_sol] = drone_planner(inst, candidate_sol);
        if (!master_check(inst, planned_sol, false))
        {
            return false;
        }

        candidate = Candidate{planned_sol, score};
        return true;
    }

    const int drone = encoded_position - truck_insert_slots;
    auto [assigned_ok, assigned_sol] = greedy_assign_launch_and_land(
        inst,
        candidate_sol,
        delivery,
        drone);
    if (!assigned_ok || !master_check(inst, assigned_sol, false))
    {
        return false;
    }

    candidate = Candidate{assigned_sol, objective_function_impl(inst, assigned_sol)};
    return true;
}
}

void generate_and_keep_top_p(
    int n,
    int m,
    int p,
    std::vector<int> &insert_positions,
    const std::vector<int> &to_insert,
    const Instance &inst,
    const Solution &sol,
    std::priority_queue<Candidate, std::vector<Candidate>, MaxHeapCompare> &top_p_heap)
{
    if ((int)(insert_positions.size()) == m)
    {
        Candidate cand = generate_new_candidate(
            inst,
            sol,
            to_insert,
            insert_positions
        );

        if (!master_check(inst, cand.sol, false))
        {
            return;
        }

        if ((int)(top_p_heap.size()) < p)
        {
            top_p_heap.push(cand);
        }
        else if (cand.score < top_p_heap.top().score)
        {
            top_p_heap.pop();
            top_p_heap.push(cand);
        }

        return;
    }

    for (int value = 0; value < n; ++value)
    {
        insert_positions.push_back(value);
        generate_and_keep_top_p(n, m, p, insert_positions, to_insert, inst, sol, top_p_heap);
        insert_positions.pop_back();
    }
}

std::vector<Candidate> heap_to_sorted_vector(
    std::priority_queue<Candidate, std::vector<Candidate>, MaxHeapCompare> heap)
{
    std::vector<Candidate> result;
    result.reserve(heap.size());
    while (!heap.empty())
    {
        result.push_back(heap.top());
        heap.pop();
    }

    std::sort(result.begin(), result.end(),
              [](const Candidate& a, const Candidate& b) {
                  return a.score < b.score;
              });

    return result;
}

bool greedy_insert(const Instance &inst, Solution &sol, std::vector<int> to_insert, int k)
{
    // k not in use
    (void)k;

    if (to_insert.empty())
    {
        return true;
    }

    const int p = 4;

    std::vector<int> current;
    current.reserve(to_insert.size());

    std::priority_queue<Candidate, std::vector<Candidate>, MaxHeapCompare> top_p_heap;
    const int insertion_targets = sol.truck_route.size() + sol.drones.size();
    generate_and_keep_top_p(
        insertion_targets,
        to_insert.size(),
        p,
        current,
        to_insert,
        inst,
        sol,
        top_p_heap
    );

    if (top_p_heap.empty())
    {
        return false;
    }

    const std::vector<Candidate> best = heap_to_sorted_vector(top_p_heap);
    const int chosen_idx = roulette_wheel_selection_exponential(best.size());
    sol = best[chosen_idx].sol;
    return true;
}

bool cheapest_feasible_sequential_insert(
    const Instance &inst,
    Solution &sol,
    std::vector<int> to_insert,
    int k)
{
    (void)k;

    while (!to_insert.empty())
    {
        const bool base_has_drone_deliveries = has_any_drone_deliveries(sol);
        const int insertion_targets = sol.truck_route.size() + sol.drones.size();

        bool found = false;
        int best_node_idx = -1;
        long long best_score = std::numeric_limits<long long>::max();
        Candidate best_candidate;

        for (int node_idx = 0; node_idx < (int)(to_insert.size()); ++node_idx)
        {
            const int delivery = to_insert[node_idx];
            for (int encoded_position = 0; encoded_position < insertion_targets; ++encoded_position)
            {
                Candidate candidate;
                if (!evaluate_single_insert_candidate(
                        inst,
                        sol,
                        delivery,
                        encoded_position,
                        base_has_drone_deliveries,
                        candidate))
                {
                    continue;
                }

                if (!found || candidate.score < best_score)
                {
                    found = true;
                    best_score = candidate.score;
                    best_node_idx = node_idx;
                    best_candidate = std::move(candidate);
                }
            }
        }

        if (!found)
        {
            return false;
        }

        sol = std::move(best_candidate.sol);
        to_insert[best_node_idx] = to_insert.back();
        to_insert.pop_back();
    }

    return true;
}