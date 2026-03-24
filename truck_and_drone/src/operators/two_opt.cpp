#include "operators/two_opt.h"
#include <algorithm>
#include <utility>

bool two_opt(const Instance &inst, Solution &solution, int first, int second)
{
    (void)inst;

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

    return two_opt(inst, solution, best_first, best_second);
}

