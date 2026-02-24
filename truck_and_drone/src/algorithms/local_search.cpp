#include "algorithms/local_search.h"
#include "verification/solution.h"
#include "verification/feasibility_check.h"
#include "operators/operator.h"
#include "datahandling/instance.h"

Solution local_search(const Instance& instance,
                      const Solution& initial,
                      std::function<bool(const Instance&, Solution&)> op,
                      std::function<long long(const Instance&, const Solution&)> objective) {

    Solution current = initial;
    long long best_cost = objective(instance, current);

    int amnt_iterations = 0;

    while (amnt_iterations < 10000) {
        amnt_iterations++;

        Solution candidate = current;           // copy current solution
        bool success = op(instance, candidate); // apply operator in-place
        if (!success) continue;                 // skip if move was invalid

        long long cost = objective(instance, candidate);
        if (cost < best_cost && master_check(instance, candidate, false)) {
            best_cost = cost;
            current = candidate; // accept improvement
        }
    }

    return current;
}