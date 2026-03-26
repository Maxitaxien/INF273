#include "operators/two_opt.h"
#include "operators/solution_fixers.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <algorithm>
#include <utility>
#include <vector>

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

