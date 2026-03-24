#include "operators/nearest_neighbour_reassign.h"
#include "datahandling/instance.h"
#include "operators/helpers.h"
#include "verification/feasibility_check.h"
#include <algorithm>
#include <utility>

bool nearest_neighbour_reassign(const Instance &inst, Solution &sol, int i) {
    // Must have at least two customers beyond the depot at route[0].
    // This ensures we can safely access i+1.
    if (sol.truck_route.size() <= 2)
        return false;

    // Guard index bounds explicitly
    if (i <= 0 || i >= (int)sol.truck_route.size() - 1)
        return false;

    // `i` is an index in the truck route; convert to the node id.
    int point_node = sol.truck_route[i];

    std::vector<int> closest = sort_by_distance_to_point_truck(inst, sol, point_node);
    if (closest.empty())
        return false;

    int candidate = closest[0];

    // If already assigned to closest point, try the second-closest (if available)
    if (candidate == sol.truck_route[i + 1] && closest.size() > 1) {
        candidate = closest[1];
    }

    Solution candidate_solution = sol;
    auto swap_position = std::find(candidate_solution.truck_route.begin(),
                                   candidate_solution.truck_route.end(),
                                   candidate);
    if (swap_position == candidate_solution.truck_route.end())
        return false;
    std::iter_swap(candidate_solution.truck_route.begin() + i, swap_position);

    if (!master_check(inst, candidate_solution, false))
        return false;

    sol = std::move(candidate_solution);
    return true;
}
