#include "algorithms/nearest_neighbour.h"

#include "general/roulette_wheel_selection.h"

#include <algorithm>
#include <utility>
#include <vector>

namespace
{
constexpr long long INF = 4e18;

Solution build_truck_only_nearest_neighbour(
    const Instance &problem_instance,
    bool randomized_shortlist)
{
    Solution solution;
    solution.truck_route.reserve(problem_instance.n + 1);
    solution.truck_route.push_back(0);

    std::vector<bool> visited(problem_instance.n + 1, false);
    visited[0] = true;

    while ((int)solution.truck_route.size() <= problem_instance.n)
    {
        const int current = solution.truck_route.back();
        std::vector<std::pair<long long, int>> candidates;
        candidates.reserve(problem_instance.n);

        for (int customer = 1; customer <= problem_instance.n; ++customer)
        {
            if (!visited[customer])
            {
                candidates.emplace_back(
                    problem_instance.truck_matrix[current][customer],
                    customer);
            }
        }

        if (candidates.empty())
        {
            break;
        }

        std::sort(candidates.begin(), candidates.end());

        int next_node = candidates.front().second;
        if (randomized_shortlist)
        {
            const int top_k = std::min(4, (int)candidates.size());
            const int shortlist_idx =
                roulette_wheel_selection_exponential(top_k, 0.28);
            next_node = candidates[(size_t)shortlist_idx].second;
        }

        solution.truck_route.push_back(next_node);
        visited[next_node] = true;
    }

    solution.drones.resize(2);
    return solution;
}
} // namespace

Solution nearest_neighbour(const Instance &problem_instance)
{
    return build_truck_only_nearest_neighbour(problem_instance, false);
}

Solution roulette_nearest_neighbour(const Instance &problem_instance)
{
    return build_truck_only_nearest_neighbour(problem_instance, true);
}
