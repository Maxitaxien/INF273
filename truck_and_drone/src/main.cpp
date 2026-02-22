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
#include "runners/all_blind_random.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"

using namespace datasets;

int main() {
    //all_blind_random();
    Instance instance = read_instance(datasets::contest);
    Solution initial = nearest_neighbour(instance);
    Solution drone = greedy_drone_cover(instance, initial);

    std::cout << convert_to_submission(drone) << "\n";

    Solution result = simulated_annealing(instance, simple_initial_solution(instance.n), one_reinsert_random, 0.1, calculate_total_waiting_time);




    /*
    Solution result = local_search(instance, simple_initial_solution(instance.n), one_reinsert_operator, calculate_total_waiting_time);
    */

    std::cout << convert_to_submission(result) << "\n";

    std::cout << master_check(instance, result, true) << "\n";
    std::cout << calculate_total_waiting_time(instance, result) << "\n";

    // now run local search from greedy drone cover!
    /*
    Solution result2 = local_search(instance, drone, one_reinsert_operator, calculate_total_waiting_time);

    std::cout << convert_to_submission(result2) << "\n";

    std::cout << master_check(instance, result2, true) << "\n";
    std::cout << calculate_total_waiting_time(instance, result2) << "\n";
    */
    
}