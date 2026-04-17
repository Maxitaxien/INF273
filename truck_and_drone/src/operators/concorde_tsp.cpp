#include "operators/operator.h"

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
}

bool concorde_linkern_improve(const Instance &instance, Solution &sol)
{
    if ((int)(sol.truck_route.size()) < 3 || sol.truck_route.front() != 0)
    {
        return false;
    }

    const long long current_cost = objective_function_impl(instance, sol);

    LinkernSolver solver(instance.truck_matrix);
    const std::vector<int> customer_nodes(
        sol.truck_route.begin() + 1,
        sol.truck_route.end());
    const LinkernTour optimized = solver.solve_route(customer_nodes);
    const std::vector<int> optimized_route =
        best_depot_path_from_customer_cycle(instance, optimized.tour, sol.truck_route);

    if (optimized_route == sol.truck_route)
    {
        return false;
    }

    Solution candidate = sol;
    candidate.truck_route = optimized_route;
    candidate = fix_overall_feasibility(instance, candidate);
    if (!master_check(instance, candidate, false))
    {
        return false;
    }

    const long long candidate_cost = objective_function_impl(instance, candidate);
    if (candidate_cost >= current_cost)
    {
        return false;
    }

    sol = std::move(candidate);
    return true;
}
