#include "algorithms/gam_escape_algorithm.h"
#include "general/roulette_wheel_selection.h"
#include "operators/operator.h"
#include "verification/objective_value.h"
#include "verification/solution.h"

#include <utility>
#include <vector>

namespace
{
bool same_solution(const Solution &lhs, const Solution &rhs)
{
    if (lhs.truck_route != rhs.truck_route || lhs.drones.size() != rhs.drones.size())
    {
        return false;
    }

    for (int drone = 0; drone < (int)(lhs.drones.size()); ++drone)
    {
        const DroneCollection &lhs_collection = lhs.drones[drone];
        const DroneCollection &rhs_collection = rhs.drones[drone];
        if (lhs_collection.launch_indices != rhs_collection.launch_indices ||
            lhs_collection.deliver_nodes != rhs_collection.deliver_nodes ||
            lhs_collection.land_indices != rhs_collection.land_indices)
        {
            return false;
        }
    }

    return true;
}
}

GAMEscapeResult gam_escape_algorithm(
    const Instance& inst, 
    Solution incumbent, 
    const std::vector<NamedOperator> &ops, 
    const std::vector<double> &selection_weights,
    int amnt_iter,
    GAMSolutionCache *cache,
    GAMFeasibilityMode feasibility_mode
) {
    const GAMSolutionEvaluation initial_evaluation =
        evaluate_solution_with_cache(
            inst,
            incumbent,
            cache,
            feasibility_mode);
    const long long initial_cost =
        initial_evaluation.objective_known
            ? initial_evaluation.objective
            : objective_function_impl(inst, incumbent);

    if (cache != nullptr && !initial_evaluation.objective_known)
    {
        gam_cache_known_feasible_solution(*cache, inst, incumbent, initial_cost);
    }

    if (ops.empty() || amnt_iter <= 0)
    {
        Solution best_seen = incumbent;
        return GAMEscapeResult{
            std::move(incumbent),
            initial_cost,
            std::move(best_seen),
            initial_cost,
            false};
    }

    Solution best = incumbent;
    long long incumbent_cost = initial_cost;
    long long best_cost = initial_cost;
    bool found_new_best = false;

    for (int i = 0; i < amnt_iter; i ++) {
        const int selected_idx = roulette_wheel_selection(selection_weights);
        Solution neighbour = incumbent;

        if (ops[selected_idx].op(inst, neighbour) &&
            !same_solution(neighbour, incumbent)) {
            const GAMSolutionEvaluation evaluation =
                evaluate_solution_with_cache(
                    inst,
                    neighbour,
                    cache,
                    feasibility_mode);

            if (!evaluation.feasible || !evaluation.objective_known)
            {
                continue;
            }

            incumbent = std::move(neighbour);
            incumbent_cost = evaluation.objective;
            if (incumbent_cost < best_cost)
            {
                best = incumbent;
                best_cost = incumbent_cost;
                found_new_best = true;
            }
        }
    }

    return GAMEscapeResult{
        std::move(incumbent),
        incumbent_cost,
        std::move(best),
        best_cost,
        found_new_best};
}
