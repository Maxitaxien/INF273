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
    Instance instance = read_instance(datasets::contest);
    Solution random = random_valid_solution(instance.n);

    bool feasible = master_check(instance, random, false);
    std::cout << feasible << "\n";

    Solution truck = nearest_neighbour(instance);

    Solution drone = greedy_drone_cover(instance, truck);

    for (int i : drone.drones[0].launch_indices) {
        std::cout << i << "\n";
    }

    std::cout << convert_to_submission(truck) << "\n";
    std::cout << convert_to_submission(drone) << "\n";

    std::cout << master_check(instance, drone, false) << "\n";

}