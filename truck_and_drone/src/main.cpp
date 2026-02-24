#include <iostream>
#include <chrono>
#include "datahandling/reader.h"
#include "datahandling/convert_to_submission.h"
#include "datahandling/datasets.h"
#include "datahandling/save_to_csv.h"
#include "algorithms/simple_initial_solution.h"
#include "algorithms/nearest_neighbour.h"
#include "algorithms/greedy_drone_cover.h"
#include "algorithms/random_valid_solution.h"
#include "algorithms/blind_random_search.h"
#include "algorithms/local_search.h"
#include "algorithms/simulated_annealing.h"
#include "operators/operator.h"
#include "operators/one_reinsert.h"
#include "runners/run_algorithm.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <map>

using namespace datasets;

int main() {
    std::map<std::string, Algorithm> algos = {
        {"Random Search", blind_random_wrapper},
        {"Local Search 1-insert", local_search_wrapper},
        {"Simulated Annealing 1-insert", sa_wrapper},
    };
    for (const auto& [name, wrapper] : algos) {
        run_algorithm(name, wrapper, one_reinsert_random, objective_function_impl);
    }
    
    /*
    
    Instance instance = read_instance(contest);
    Solution initial = nearest_neighbour(instance);

    Solution drone = greedy_drone_cover(instance, initial);

    long long res = objective_function_impl(instance, drone);
    std::cout << res << "\n";
    */
}