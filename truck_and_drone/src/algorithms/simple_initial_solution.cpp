#include "algorithms/simple_initial_solution.h"
#include <numeric>

Solution simple_initial_solution(int n) {
    Solution simple_solution;

    simple_solution.truck_route.emplace_back(0);
    std::vector<int> numbers(n);
    std::iota(numbers.begin(), numbers.end(), 1);
    
    for (int val: numbers) {
        simple_solution.truck_route.emplace_back(val);
    }

    return simple_solution;
}