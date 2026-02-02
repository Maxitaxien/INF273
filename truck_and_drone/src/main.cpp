#include <iostream>
#include "datahandling/reader.h"
#include "datahandling/convert_to_submission.h"
#include "datahandling/datasets.h"
#include "algorithms/nearest_neighbour.h"
#include "algorithms/greedy_drone_cover.h"
#include "verification/feasibility_check.h"

int main() {
    Instance problem_instance = read_instance(datasets::contest);
    Solution initial_solution = nearest_neighbour(problem_instance);

    Solution drone_cover_solution = greedy_drone_cover(problem_instance, initial_solution);

    std::string submission1 = convert_to_submission(initial_solution);
    std::string submission2 = convert_to_submission(drone_cover_solution);
    
    std::cout << submission1 << "\n";
    std::cout << submission2 << "\n";
    std::cout << includes_all_nodes(problem_instance.n, submission2) << "\n";

    std::cout << all_drone_flights_under_lim(problem_instance, drone_cover_solution);
}