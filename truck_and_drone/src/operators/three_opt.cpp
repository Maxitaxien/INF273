#include "operators/three_opt.h"
#include "general/random.h"
#include "operators/customer_slot_helpers.h"
#include "operators/drone_planner.h"
#include "operators/operator.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <algorithm>
#include <array>
#include <numeric>
#include <vector>

namespace
{

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

std::vector<int> reversed_copy(const std::vector<int> &segment)
{
    std::vector<int> result = segment;
    std::reverse(result.begin(), result.end());
    return result;
}

std::vector<int> concat_segments(
    const std::vector<int> &prefix,
    const std::vector<int> &middle_a,
    const std::vector<int> &middle_b,
    const std::vector<int> &suffix)
{
    std::vector<int> route;
    route.reserve(prefix.size() + middle_a.size() + middle_b.size() + suffix.size());
    route.insert(route.end(), prefix.begin(), prefix.end());
    route.insert(route.end(), middle_a.begin(), middle_a.end());
    route.insert(route.end(), middle_b.begin(), middle_b.end());
    route.insert(route.end(), suffix.begin(), suffix.end());
    return route;
}

bool valid_breakpoints(const Solution &sol, int first, int second, int third)
{
    const int route_size = (int)(sol.truck_route.size());
    return route_size >= 4 &&
        0 <= first && first < second && second < third &&
        third < route_size;
}

std::vector<std::vector<int>> generate_three_opt_candidates(
    const Solution &sol,
    int first,
    int second,
    int third)
{
    const std::vector<int> prefix(
        sol.truck_route.begin(),
        sol.truck_route.begin() + first + 1);
    const std::vector<int> segment_a(
        sol.truck_route.begin() + first + 1,
        sol.truck_route.begin() + second + 1);
    const std::vector<int> segment_b(
        sol.truck_route.begin() + second + 1,
        sol.truck_route.begin() + third + 1);
    const std::vector<int> suffix(
        sol.truck_route.begin() + third + 1,
        sol.truck_route.end());

    std::vector<std::vector<int>> candidates;
    candidates.reserve(7);

    candidates.push_back(concat_segments(prefix, reversed_copy(segment_a), segment_b, suffix));
    candidates.push_back(concat_segments(prefix, segment_a, reversed_copy(segment_b), suffix));
    candidates.push_back(concat_segments(prefix, reversed_copy(segment_a), reversed_copy(segment_b), suffix));
    candidates.push_back(concat_segments(prefix, segment_b, segment_a, suffix));
    candidates.push_back(concat_segments(prefix, segment_b, reversed_copy(segment_a), suffix));
    candidates.push_back(concat_segments(prefix, reversed_copy(segment_b), segment_a, suffix));
    candidates.push_back(concat_segments(prefix, reversed_copy(segment_b), reversed_copy(segment_a), suffix));

    return candidates;
}
}

bool three_opt(const Instance &inst, Solution &sol, int first, int second, int third)
{
    if (!valid_breakpoints(sol, first, second, third))
    {
        return false;
    }

    const long long initial_truck_cost =
        objective_function_truck_only(inst, sol.truck_route);
    long long best_truck_cost = initial_truck_cost;
    std::vector<int> best_route;

    for (const std::vector<int> &candidate :
         generate_three_opt_candidates(sol, first, second, third))
    {
        const long long candidate_cost =
            objective_function_truck_only(inst, candidate);
        if (candidate_cost < best_truck_cost)
        {
            best_truck_cost = candidate_cost;
            best_route = candidate;
        }
    }

    if (best_route.empty())
    {
        return false;
    }

    sol.truck_route = std::move(best_route);
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
