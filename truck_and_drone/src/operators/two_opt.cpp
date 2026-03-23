#include "operators/two_opt.h"
#include "operators/solution_fixers.h"
#include "verification/solution.h"
#include <vector>

void two_opt(const Instance &inst, Solution &solution, int first, int second)
{
    const int route_size = (int)(solution.truck_route.size());
    std::vector<int> new_truck_route(route_size);

    // 1: Add all up to and including first position
    for (int i = 0; i <= first; i++)
    {
        new_truck_route[i] = solution.truck_route[i];
    }

    // 2: Add route[first+1] to route[second] in reverse order
    int idx = first + 1;
    for (int j = second; j > first; j--)
    {
        new_truck_route[idx++] = solution.truck_route[j];
    }

    // 3: Add the rest
    for (int k = second + 1; k < route_size; k++)
    {
        new_truck_route[k] = solution.truck_route[k];
    }

    solution.truck_route = new_truck_route;
    solution = fix_overall_feasibility(inst, solution);
}

