#include "solution_fixers/solution_fixers.h"

#include "operators/drone_planner.h"
#include "verification/feasibility_check.h"

Solution fix_overall_feasibility(const Instance &instance, Solution &solution)
{
    simple_fix_validity(solution);
    if (master_check(instance, solution, false))
    {
        return solution;
    }

    const auto [planner_score, planned_solution] = drone_planner(instance, solution);
    (void)planner_score;
    if (master_check(instance, planned_solution, false))
    {
        return planned_solution;
    }

    Solution fallback = solution;
    for (int drone = 0; drone < (int)fallback.drones.size(); ++drone)
    {
        fix_feasibility_for_drone(instance, fallback, drone);
    }
    simple_fix_validity(fallback);
    return fallback;
}
