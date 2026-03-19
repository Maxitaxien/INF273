#include "algorithms/local_search.h"
#include "verification/solution.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include "operators/operator.h"
#include "datahandling/instance.h"
#include <vector>

Solution local_search(const Instance &instance,
                      const Solution &initial,
                      Operator op)
{

    Solution current = initial;
    long long best_cost = objective_function_impl(instance, current);

    int amnt_iterations = 0;
    Operator current_operator;

    while (amnt_iterations < 10000)
    {
        amnt_iterations++;

        Solution candidate = current;           // copy current solution
        bool success = op(instance, candidate); // apply operator in-place
        if (!success)
            continue; // skip if move was invalid

        if (!master_check(instance, candidate, false))
            continue;

        long long cost = objective_function_impl(instance, candidate);
        if (cost < best_cost)
        {
            best_cost = cost;
            current = candidate; // accept improvement
        }
    }

    return current;
}
