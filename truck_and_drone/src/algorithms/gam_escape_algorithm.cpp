#include "algorithms/gam_escape_algorithm.h"
#include "verification/solution.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include "general/roulette_wheel_selection.h"
#include "operators/operator.h"

#include <vector>

Solution gam_escape_algorithm(
    const Instance& inst, 
    Solution incumbent, 
    const std::vector<NamedOperator> &ops, 
    const std::vector<double> &selection_weights,
    int amnt_iter
) {
    if (ops.empty() || amnt_iter <= 0)
    {
        return incumbent;
    }

    Solution best = incumbent;
    long long best_cost = objective_function_impl(inst, incumbent);

    for (int i = 0; i < amnt_iter; i ++) {
        const int selected_idx = roulette_wheel_selection(selection_weights);
        Solution neighbour = incumbent;

        if (ops[selected_idx].op(inst, neighbour) && master_check(inst, neighbour, false)) {
            incumbent = neighbour;
            const long long incumbent_cost = objective_function_impl(inst, incumbent);
            if (incumbent_cost < best_cost)
            {
                best = incumbent;
                best_cost = incumbent_cost;
            }
        }
    } 
    return best;
}
