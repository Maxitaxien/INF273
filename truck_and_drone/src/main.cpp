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
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"

const long long INF = 4e18;

using namespace datasets;

int main() {
    Instance instance = read_instance(datasets::contest2);
    Solution initial = nearest_neighbour(instance);
    Solution drone_cover = greedy_drone_cover(instance, initial);

    bool feasible = master_check(instance, drone_cover, true);
    std::cout << feasible << "\n";

    std::string sub = convert_to_submission(drone_cover);

    std::cout << sub << "\n";
}