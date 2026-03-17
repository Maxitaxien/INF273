#include "algorithms/gam.h"
#include "algorithms/simple_initial_solution.h"
#include "datahandling/instance.h"
#include "verification/solution.h"
#include "runners/algorithms.h"
#include <vector>

Solution general_adaptive_metaheuristic_alns(
    const Instance inst,
    std::vector<RemovalHeuristic> remove,
    std::vector<InsertionHeuristic> insert,
    Algorithm escape_algorithm,
    int escape_condition)
{
    Solution curr_solution = simple_initial_solution(inst.n);
    Solution best = curr_solution;

    int non_improving_iter = 0;
    int amnt_iterations = 0;

    while (amnt_iterations < 10000)
    {
        if (non_improving_iter > escape_condition)
        {
            curr_solution = escape_algorithm(curr_solution);
        }

        amnt_iterations++;
    }
}