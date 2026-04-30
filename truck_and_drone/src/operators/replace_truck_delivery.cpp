#include "operators/replace_truck_delivery.h"
#include "datahandling/instance.h"
#include "general/roulette_wheel_selection.h"
#include "general/sort_drone_collection.h"
#include "operators/helpers.h"
#include "solution_fixers/solution_fixers.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <algorithm>
#include <limits>
#include <random>
#include <utility>

extern std::mt19937 gen;

namespace
{
constexpr size_t kTopGreedyCandidates = 5;

struct ScoredTruckReplacementCandidate
{
    long long cost = std::numeric_limits<long long>::max();
    Solution solution;
};

void insert_top_candidate(
    std::vector<ScoredTruckReplacementCandidate> &top_candidates,
    Solution candidate,
    long long cost)
{
    auto insert_it = top_candidates.begin();
    while (insert_it != top_candidates.end() && insert_it->cost <= cost)
    {
        ++insert_it;
    }

    top_candidates.insert(
        insert_it,
        ScoredTruckReplacementCandidate{
            cost,
            std::move(candidate),
        });

    if (top_candidates.size() > kTopGreedyCandidates)
    {
        top_candidates.pop_back();
    }
}
}

bool replace_truck_delivery(const Instance &inst, Solution &sol, int idx, int drone)
{
    if (idx <= 0 || idx >= (int)(sol.truck_route.size()))
    {
        return false;
    }
    if (drone < 0 || drone >= (int)(sol.drones.size()))
    {
        return false;
    }

    const int customer = sol.truck_route[idx];
    const std::vector<AffectedDroneFlight> affected =
        collect_removed_anchor_drone_flights(sol, idx);

    Solution candidate = sol;
    pop_truck_delivery(candidate, idx);

    if (!repair_affected_drone_flights_localized(
            inst,
            sol,
            affected,
            candidate,
            std::vector<int>{customer}))
    {
        return false;
    }

    auto [success, ignored_solution] = greedy_assign_launch_and_land_assume_valid(
        inst,
        candidate,
        customer,
        drone);
    (void)ignored_solution;
    if (!success || !master_check(inst, candidate, false))
    {
        return false;
    }

    sol = std::move(candidate);
    return true;
}

bool replace_truck_delivery_random(const Instance &instance, Solution &sol)
{
    std::vector<std::pair<int, int>> candidates;
    for (int drone = 0; drone < (int)(sol.drones.size()); ++drone)
    {
        for (int idx = 1; idx < (int)(sol.truck_route.size()); ++idx)
        {
            candidates.emplace_back(drone, idx);
        }
    }

    if (candidates.empty())
    {
        return false;
    }

    std::shuffle(candidates.begin(), candidates.end(), gen);
    const int max_attempts = std::min(10, (int)(candidates.size()));
    Solution candidate = sol;

    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
        const auto [drone, idx] = candidates[attempt];
        candidate = sol;
        if (replace_truck_delivery(instance, candidate, idx, drone))
        {
            sol = std::move(candidate);
            return true;
        }
    }

    return false;
}

bool replace_truck_delivery_greedy(const Instance &instance, Solution &sol)
{
    std::vector<ScoredTruckReplacementCandidate> top_candidates;
    top_candidates.reserve(kTopGreedyCandidates);
    Solution candidate = sol;

    for (int drone = 0; drone < (int)sol.drones.size(); ++drone)
    {
        for (int i = 1; i < (int)sol.truck_route.size(); ++i)
        {
            candidate = sol;
            if (!replace_truck_delivery(instance, candidate, i, drone))
            {
                continue;
            }

            const long long cost = objective_function_impl(instance, candidate);
            insert_top_candidate(top_candidates, std::move(candidate), cost);
        }
    }

    if (!top_candidates.empty())
    {
        const int selected_idx = roulette_wheel_selection_exponential(
            (int)top_candidates.size());
        sol = std::move(top_candidates[(size_t)selected_idx].solution);
        return true;
    }

    return false;
}

