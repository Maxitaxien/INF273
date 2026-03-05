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
#include <filesystem>

using namespace datasets;
namespace fs = std::filesystem;

int main()
{
    run_all_algos();

    /*

    Instance instance = read_instance(contest);
    Solution initial = nearest_neighbour(instance);

    Solution drone = greedy_drone_cover(instance, initial);

    long long res = objective_function_impl(instance, drone);
    std::cout << res << "\n";
    */
}