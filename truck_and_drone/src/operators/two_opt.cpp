#include "operators/two_opt.h"
#include "general/random.h"
#include "operators/drone_planner.h"
#include "operators/operator.h"
#include "operators/solution_fixers.h"
#include "operators/customer_slot_helpers.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <algorithm>
#include <numeric>
#include <utility>
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
}

bool two_opt(const Instance &inst, Solution &solution, int first, int second)
{
    const int route_size = (int)(solution.truck_route.size());
    if (route_size < 4)
    {
        return false;
    }

    if (first > second)
    {
        std::swap(first, second);
    }

    if (first <= 0 || second >= route_size || second <= first + 1)
    {
        return false;
    }

    Solution candidate = solution;
    std::reverse(candidate.truck_route.begin() + first + 1,
                 candidate.truck_route.begin() + second + 1);

    if (!master_check(inst, candidate, false))
    {
        candidate = fix_overall_feasibility(inst, candidate);
        if (!master_check(inst, candidate, false))
        {
            return false;
        }
    }

    solution = std::move(candidate);
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

bool two_opt_greedy(const Instance &inst, Solution &solution)
{
    const int route_size = (int)(solution.truck_route.size());
    if (route_size < 5)
    {
        return false;
    }

    long long best_gain = 0;
    int best_first = -1;
    int best_second = -1;

    for (int first = 1; first <= route_size - 4; ++first)
    {
        const int a = solution.truck_route[first];
        const int b = solution.truck_route[first + 1];

        for (int second = first + 2; second <= route_size - 2; ++second)
        {
            const int c = solution.truck_route[second];
            const int d = solution.truck_route[second + 1];
            const long long gain =
                inst.truck_matrix[a][b] +
                inst.truck_matrix[c][d] -
                inst.truck_matrix[a][c] -
                inst.truck_matrix[b][d];

            if (gain > best_gain)
            {
                best_gain = gain;
                best_first = first;
                best_second = second;
            }
        }
    }

    if (best_gain <= 0)
    {
        return false;
    }

    const long long current_cost = objective_function_impl(inst, solution);
    Solution candidate = solution;
    if (!two_opt(inst, candidate, best_first, best_second))
    {
        return false;
    }

    const long long candidate_cost = objective_function_impl(inst, candidate);
    if (candidate_cost >= current_cost)
    {
        return false;
    }

    solution = std::move(candidate);
    return true;
}

bool two_opt_first_improvement(const Instance &inst, Solution &solution)
{
    const int route_size = (int)(solution.truck_route.size());
    if (route_size < 5)
    {
        return false;
    }

    const long long current_cost = objective_function_impl(inst, solution);

    for (int first = 1; first <= route_size - 4; ++first)
    {
        const int a = solution.truck_route[first];
        const int b = solution.truck_route[first + 1];

        for (int second = first + 2; second <= route_size - 2; ++second)
        {
            const int c = solution.truck_route[second];
            const int d = solution.truck_route[second + 1];
            const long long gain =
                inst.truck_matrix[a][b] +
                inst.truck_matrix[c][d] -
                inst.truck_matrix[a][c] -
                inst.truck_matrix[b][d];

            if (gain <= 0)
            {
                continue;
            }

            Solution candidate = solution;
            if (!two_opt(inst, candidate, first, second))
            {
                continue;
            }

            const long long candidate_cost = objective_function_impl(inst, candidate);
            if (candidate_cost < current_cost)
            {
                solution = std::move(candidate);
                return true;
            }
        }
    }

    return false;
}
