#include "algorithms/gam_solution_cache.h"

#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include "verification/solution.h"

#include <vector>

namespace
{
void append_int_vector(std::string &buffer, const std::vector<int> &values)
{
    buffer += std::to_string(values.size());
    buffer.push_back(':');

    for (int value : values)
    {
        buffer += std::to_string(value);
        buffer.push_back(',');
    }
}
} // namespace

std::string gam_solution_cache_key(const Solution &solution)
{
    std::string key;
    key.reserve(
        solution.truck_route.size() * 6 +
        solution.drones.size() * 16 +
        32);

    append_int_vector(key, solution.truck_route);
    key.push_back('|');
    key += std::to_string(solution.drones.size());
    key.push_back('|');

    for (const DroneCollection &collection : solution.drones)
    {
        append_int_vector(key, collection.launch_indices);
        key.push_back('|');
        append_int_vector(key, collection.deliver_nodes);
        key.push_back('|');
        append_int_vector(key, collection.land_indices);
        key.push_back('#');
    }

    return key;
}

GAMCachedSolution &gam_solution_cache_entry(
    GAMSolutionCache &cache,
    const Solution &solution,
    bool *is_new_solution)
{
    const auto [it, inserted] = cache.try_emplace(gam_solution_cache_key(solution));
    if (is_new_solution != nullptr)
    {
        *is_new_solution = inserted;
    }

    return it->second;
}

void gam_cache_known_feasible_solution(
    GAMSolutionCache &cache,
    const Instance &instance,
    const Solution &solution,
    long long objective)
{
    const Solution canonical =
        canonicalize_terminal_depot_landings(instance, solution);
    bool unused_is_new = false;
    GAMCachedSolution &entry =
        gam_solution_cache_entry(cache, canonical, &unused_is_new);
    entry.feasible_known = true;
    entry.feasible = true;
    entry.objective_known = true;
    entry.objective = objective;
}

GAMSolutionEvaluation evaluate_solution_with_cache(
    const Instance &instance,
    const Solution &solution,
    GAMSolutionCache *cache)
{
    GAMSolutionEvaluation evaluation;

    if (cache == nullptr)
    {
        // TEST: Assume feasible. 
        // evaluation.feasible = master_check(instance, solution, false);
        evaluation.feasible = true;
        if (evaluation.feasible)
        {
            evaluation.objective_known = true;
            evaluation.objective = objective_function_impl(instance, solution);
        }

        return evaluation;
    }

    const Solution canonical =
        canonicalize_terminal_depot_landings(instance, solution);
    GAMCachedSolution &entry =
        gam_solution_cache_entry(*cache, canonical, &evaluation.is_new_solution);

    if (!entry.feasible_known)
    {
        // TEST: Assume feasible
        entry.feasible = true;
        // entry.feasible = master_check(instance, canonical, false);
        entry.feasible_known = true;
    }

    evaluation.feasible = entry.feasible;

    if (entry.feasible)
    {
        if (!entry.objective_known)
        {
            entry.objective = objective_function_impl(instance, canonical);
            entry.objective_known = true;
        }

        evaluation.objective_known = true;
        evaluation.objective = entry.objective;
    }

    return evaluation;
}
