#include "operators/two_opt.h"
#include "general/random.h"
#include "operators/drone_planner.h"
#include "operators/operator.h"
#include "operators/solution_fixers.h"
#include "operators/customer_slot_helpers.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <algorithm>
#include <limits>
#include <numeric>
#include <tuple>
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

long long truck_arrival_sum_only(
    const Instance &inst,
    const std::vector<int> &route)
{
    long long total_arrival = 0;
    long long arrival = 0;
    for (int idx = 1; idx < (int)(route.size()); ++idx)
    {
        arrival += inst.truck_matrix[route[idx - 1]][route[idx]];
        total_arrival += arrival;
    }

    return total_arrival;
}

int count_affected_drone_flights(
    const Solution &solution,
    int first,
    int second)
{
    const int reversed_start = first + 1;
    const int reversed_end = second;
    int affected = 0;

    for (const DroneCollection &drone : solution.drones)
    {
        const int flight_count = std::min(
            (int)(drone.launch_indices.size()),
            (int)(drone.land_indices.size()));
        for (int flight_idx = 0; flight_idx < flight_count; ++flight_idx)
        {
            const int launch_idx = drone.launch_indices[flight_idx];
            const int land_idx = drone.land_indices[flight_idx];
            if (launch_idx > reversed_end || land_idx < reversed_start)
            {
                continue;
            }

            ++affected;
        }
    }

    return affected;
}

struct ArrivalRankedTwoOptMove
{
    int first = -1;
    int second = -1;
    long long arrival_score = std::numeric_limits<long long>::max();
    int affected_flights = std::numeric_limits<int>::max();
};
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

bool two_opt_arrival_screened(const Instance &inst, Solution &solution)
{
    const int route_size = (int)(solution.truck_route.size());
    if (route_size < 5)
    {
        return false;
    }

    const long long current_cost = objective_function_impl(inst, solution);
    const long long current_arrival_score =
        truck_arrival_sum_only(inst, solution.truck_route);

    std::vector<ArrivalRankedTwoOptMove> promising_moves;
    promising_moves.reserve((route_size * route_size) / 2);
    std::vector<int> candidate_route = solution.truck_route;

    for (int first = 1; first <= route_size - 4; ++first)
    {
        for (int second = first + 2; second <= route_size - 2; ++second)
        {
            candidate_route = solution.truck_route;
            std::reverse(
                candidate_route.begin() + first + 1,
                candidate_route.begin() + second + 1);

            const long long arrival_score =
                truck_arrival_sum_only(inst, candidate_route);
            if (arrival_score >= current_arrival_score)
            {
                continue;
            }

            promising_moves.push_back(ArrivalRankedTwoOptMove{
                first,
                second,
                arrival_score,
                count_affected_drone_flights(solution, first, second),
            });
        }
    }

    if (promising_moves.empty())
    {
        return false;
    }

    std::sort(
        promising_moves.begin(),
        promising_moves.end(),
        [](const ArrivalRankedTwoOptMove &lhs, const ArrivalRankedTwoOptMove &rhs) {
            return std::tie(lhs.arrival_score, lhs.affected_flights, lhs.first, lhs.second) <
                std::tie(rhs.arrival_score, rhs.affected_flights, rhs.first, rhs.second);
        });

    const int max_trials = std::min(
        (int)(promising_moves.size()),
        inst.n >= 50 ? 2 : 3);
    Solution candidate = solution;

    for (int trial = 0; trial < max_trials; ++trial)
    {
        const ArrivalRankedTwoOptMove &move = promising_moves[trial];
        candidate = solution;
        if (!two_opt(inst, candidate, move.first, move.second))
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

    return false;
}
