#include "operators/three_opt.h"
#include "verification/objective_value.h"
#include <algorithm>
#include <vector>

namespace
{
std::vector<int> reversed_copy(const std::vector<int> &segment)
{
    std::vector<int> result = segment;
    std::reverse(result.begin(), result.end());
    return result;
}

std::vector<int> concat_segments(
    const std::vector<int> &prefix,
    const std::vector<int> &middle_a,
    const std::vector<int> &middle_b,
    const std::vector<int> &suffix)
{
    std::vector<int> route;
    route.reserve(prefix.size() + middle_a.size() + middle_b.size() + suffix.size());
    route.insert(route.end(), prefix.begin(), prefix.end());
    route.insert(route.end(), middle_a.begin(), middle_a.end());
    route.insert(route.end(), middle_b.begin(), middle_b.end());
    route.insert(route.end(), suffix.begin(), suffix.end());
    return route;
}

bool valid_breakpoints(const Solution &sol, int first, int second, int third)
{
    const int route_size = (int)(sol.truck_route.size());
    return route_size >= 4 &&
        0 <= first && first < second && second < third &&
        third < route_size;
}

std::vector<std::vector<int>> generate_three_opt_candidates(
    const Solution &sol,
    int first,
    int second,
    int third)
{
    const std::vector<int> prefix(
        sol.truck_route.begin(),
        sol.truck_route.begin() + first + 1);
    const std::vector<int> segment_a(
        sol.truck_route.begin() + first + 1,
        sol.truck_route.begin() + second + 1);
    const std::vector<int> segment_b(
        sol.truck_route.begin() + second + 1,
        sol.truck_route.begin() + third + 1);
    const std::vector<int> suffix(
        sol.truck_route.begin() + third + 1,
        sol.truck_route.end());

    std::vector<std::vector<int>> candidates;
    candidates.reserve(7);

    candidates.push_back(concat_segments(prefix, reversed_copy(segment_a), segment_b, suffix));
    candidates.push_back(concat_segments(prefix, segment_a, reversed_copy(segment_b), suffix));
    candidates.push_back(concat_segments(prefix, reversed_copy(segment_a), reversed_copy(segment_b), suffix));
    candidates.push_back(concat_segments(prefix, segment_b, segment_a, suffix));
    candidates.push_back(concat_segments(prefix, segment_b, reversed_copy(segment_a), suffix));
    candidates.push_back(concat_segments(prefix, reversed_copy(segment_b), segment_a, suffix));
    candidates.push_back(concat_segments(prefix, reversed_copy(segment_b), reversed_copy(segment_a), suffix));

    return candidates;
}
}

bool three_opt(const Instance &inst, Solution &sol, int first, int second, int third)
{
    if (!valid_breakpoints(sol, first, second, third))
    {
        return false;
    }

    const long long initial_truck_cost =
        objective_function_truck_only(inst, sol.truck_route);
    long long best_truck_cost = initial_truck_cost;
    std::vector<int> best_route;

    for (const std::vector<int> &candidate :
         generate_three_opt_candidates(sol, first, second, third))
    {
        const long long candidate_cost =
            objective_function_truck_only(inst, candidate);
        if (candidate_cost < best_truck_cost)
        {
            best_truck_cost = candidate_cost;
            best_route = candidate;
        }
    }

    if (best_route.empty())
    {
        return false;
    }

    sol.truck_route = std::move(best_route);
    return true;
}
