#include "algorithms/local_search.h"
#include "verification/solution.h"
#include "operators/operator.h"
#include "datahandling/instance.h"

Solution local_search(const Instance& instance,
                      const Solution& initial,
                      Operator op,
                      std::function<long long(const Instance&, const Solution&)> objective) {

    Solution current = initial;
    long long best_cost = objective(instance, current);

    int amnt_iterations = 0;
    bool improved = true;

    while (improved && amnt_iterations < 10000) {
        improved = false;
        amnt_iterations++;

        Solution s = op(instance, current);   
        
        long long cost = objective(instance, s);
        if (cost < best_cost) {
            best_cost = cost;
            current = s;
            improved = true;
        }
    }

    return current;
}