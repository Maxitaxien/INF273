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
    /*
    // Run with a single operator 
    run_all_algos(NamedOperator{"1-insert", one_reinsert_random});
    run_all_algos(NamedOperator{"NN-Reassign", nearest_neighbour_reassign_random});

    // Run with multiple operators using uniform weights
    run_all_algos(
        {NamedOperator{"1-insert", one_reinsert_random}, NamedOperator{"NN-Reassign", nearest_neighbour_reassign_random}});

    // Run with multiple operators using tuned weights
    run_all_algos(
        {NamedOperator{"1-insert", one_reinsert_random}, NamedOperator{"NN-Reassign", nearest_neighbour_reassign_random}},
        {0.7, 0.3});
    */
    run_all_algos(
    {
        NamedOperator{"Replace Truck Delivery", replace_truck_delivery_greedy}, 
        NamedOperator{"NN-Reassign", nearest_neighbour_reassign_random},
        NamedOperator{"Two-Opt", two_opt_random}
    });
    
    
    run_all_algos(
    {
        NamedOperator{"Replace Truck Delivery", replace_truck_delivery_greedy}, 
        NamedOperator{"NN-Reassign", nearest_neighbour_reassign_random},
        NamedOperator{"Two-Opt", two_opt_random}
    },
    {0.1, 0.2, 0.7}
    );
    
}