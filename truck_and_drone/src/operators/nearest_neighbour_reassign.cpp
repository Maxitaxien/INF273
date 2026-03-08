#include "operators/nearest_neighbour_reassign.h"
#include "datahandling/instance.h"
#include "verification/solution.h"
#include "operators/helpers.h"
#include "operators/solution_fixers.h"
#include <algorithm>

void nearest_neighbour_reassign(const Instance &inst, Solution &sol, int i) {
    std::vector<int> closest = sort_by_distance_to_point_truck(inst, sol, i);

    int candidate = closest[0];

    // If already assigned to closest point, do some exploration and assign to second closest instead
    if (candidate == sol.truck_route[i + 1]) {
        candidate = closest[1];
    }

    auto candidate_position = std::find(sol.truck_route.begin(), sol.truck_route.end(), candidate);

    std::iter_swap(sol.truck_route.begin() + i, candidate_position);

    sol = fix_overall_feasibility(inst, sol);
}