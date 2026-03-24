#include "operators/two_opt.h"
#include "verification/feasibility_check.h"
#include <algorithm>
#include <utility>

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
        return false;
    }

    solution = std::move(candidate);
    return true;
}

