#include <iostream>
#include "datahandling/reader.h"
#include "datahandling/datasets.h"
#include "algorithms/nearest_neighbour.h"
#include "algorithms/greedy_drone_cover.h"

int main() {
    Instance problem_instance = read_instance(datasets::toy);
    Solution initial_solution = nearest_neighbour(problem_instance);


    for (int i: initial_solution.truck_route) {
        std::cout << i << " ";
    }

    std::cout << "\n";
    Solution drone_cover_solution = greedy_drone_cover(problem_instance, initial_solution);
    for (int i: drone_cover_solution.truck_route) {
        std::cout << i << " ";
    }

    for (const auto& [key, vec] : drone_cover_solution.drone_map) {
    std::cout << "Key: " << key << "\n";
    for (const auto& tup : vec) {
        int first = std::get<0>(tup);
        int second = std::get<1>(tup);
        std::cout << "  Tuple: (" << first << ", " << second << ")\n";
    }
}

    
}