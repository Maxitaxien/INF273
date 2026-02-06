#include "algorithms/blind_random_search.h"
#include "algorithms/random_valid_solution.h"
#include "verification/feasibility_check.h"

Solution blind_random_search(
    const Instance& instance, 
    const Solution& initial, 
    std::function<long long(const Instance&, const Solution&)> objective_value_check) {
        Solution best_solution = initial;
        long long best_solution_val = objective_value_check(instance, initial);

        Solution current_solution;
        long long current_value;
        for (int i = 0; i < 10000; i++) {
            current_solution = random_valid_solution(instance.n);
            current_value = objective_value_check(instance, current_solution);            
            if (
                master_check(instance, current_solution, false) && 
                current_value < best_solution_val
            ) {
                best_solution = current_solution;
                best_solution_val = current_value;
            }
        }

        return best_solution;
    }