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
#include "operators/two_opt.h"
#include "runners/run_algorithm.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <map>
#include <filesystem>

using namespace datasets;
namespace fs = std::filesystem;

int main()
{
    // run_construction_algos();
    //  run_all_algos(one_reinsert_random);
    // run_all_algos(replace_truck_delivery_greedy);
    // run_all_algos(two_opt_random);
    run_all_algos(NamedOperator{"1-insert", one_reinsert_random});
    run_all_algos(NamedOperator{"NN-Reassign", nearest_neighbour_reassign_random});
    run_all_algos(NamedOperator{"Two-Opt", two_opt_random});
    run_all_algos(NamedOperator{"Replace Truck Delivery", replace_truck_delivery_greedy});
}