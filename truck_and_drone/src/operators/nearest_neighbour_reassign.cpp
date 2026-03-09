#include "operators/nearest_neighbour_reassign.h"
#include "datahandling/instance.h"
#include "verification/solution.h"
#include "operators/helpers.h"
#include "operators/solution_fixers.h"
#include <algorithm>

void nearest_neighbour_reassign(const Instance &inst, Solution &sol, int i) {
    // Must have at least two customers beyond the depot at route[0].
    // This ensures we can safely access i+1.
    if (sol.truck_route.size() <= 2)
        return;

    // `i` is an index in the truck route; convert to the node id.
    int point_node = sol.truck_route[i];

    std::vector<int> closest = sort_by_distance_to_point_truck(inst, sol, point_node);
    if (closest.empty())
        return;

    int candidate = closest[0];

    // If already assigned to closest point, try the second-closest (if available)
    if (candidate == sol.truck_route[i + 1] && closest.size() > 1) {
        candidate = closest[1];
    }

    auto candidate_position = std::find(sol.truck_route.begin(), sol.truck_route.end(), candidate);
    if (candidate_position == sol.truck_route.end())
        return;

    std::iter_swap(sol.truck_route.begin() + i, candidate_position);

    sol = fix_overall_feasibility(inst, sol);
}