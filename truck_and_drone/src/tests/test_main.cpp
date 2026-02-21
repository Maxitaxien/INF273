#include <iostream>
#include <cassert>
#include <vector>
#include "datahandling/reader.h"
#include "datahandling/convert_to_submission.h"
#include "datahandling/datasets.h"
#include "algorithms/simple_initial_solution.h"
#include "algorithms/nearest_neighbour.h"
#include "algorithms/greedy_drone_cover.h"
#include "operators/one_reinsert.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <vector>

#include "datahandling/instance.h"

using namespace datasets;

void test_instance_loading() {
    Instance instance = read_instance(datasets::contest);
    assert(!instance.truck_matrix.empty());
    assert(!instance.drone_matrix.empty());
    assert(instance.n > 0);
    std::cout << "✓ test_instance_loading passed\n";
}

void test_nearest_neighbour_validity() {
    Instance instance = read_instance(datasets::contest);
    Solution solution = nearest_neighbour(instance);
    assert(master_check(instance, solution, true));
    std::cout << "✓ test_nearest_neighbour_validity passed\n";
}

void test_greedy_drone_cover_validity() {
    Instance instance = read_instance(datasets::contest);
    Solution initial = nearest_neighbour(instance);
    Solution solution = greedy_drone_cover(instance, initial);
    assert(master_check(instance, solution, true));
    std::cout << "✓ test_greedy_drone_cover_validity passed\n";
}

void test_one_reinsert_validity() {
    Instance instance = read_instance(datasets::contest);
    Solution initial = nearest_neighbour(instance);
    Solution drone = greedy_drone_cover(instance, initial);
    Solution edited = one_reinsert(instance, drone, 2, 2, 12);
    assert(master_check(instance, edited, true));
    std::cout << "✓ test_one_reinsert_validity passed\n";
}

void test_one_reinsert_bounds() {
    Instance instance = read_instance(datasets::contest);
    Solution initial = nearest_neighbour(instance);
    Solution drone = greedy_drone_cover(instance, initial);
    Solution edited = one_reinsert(instance, drone, 1, 1, 5);
    assert(edited.truck_route.size() <= drone.truck_route.size() + 1);
    std::cout << "✓ test_one_reinsert_bounds passed\n";
}

void test_objective_improvement() {
    Instance instance = read_instance(datasets::contest);
    Solution initial = nearest_neighbour(instance);
    Solution drone = greedy_drone_cover(instance, initial);
    double initial_cost = calculate_total_waiting_time(instance, drone);
    Solution edited = one_reinsert(instance, drone, 2, 2, 12);
    double edited_cost = calculate_total_waiting_time(instance, edited);
    assert(edited_cost <= initial_cost * 1.01);
    std::cout << "✓ test_objective_improvement passed\n";
}

void test_solution_consistency() {
    Instance instance = read_instance(datasets::contest);
    Solution initial = nearest_neighbour(instance);
    Solution drone = greedy_drone_cover(instance, initial);
    Solution edited = one_reinsert(instance, drone, 2, 2, 12);
    assert(!convert_to_submission(edited).empty());
    std::cout << "✓ test_solution_consistency passed\n";
}

int main() {
    try {
        test_instance_loading();
        test_nearest_neighbour_validity();
        test_greedy_drone_cover_validity();
        test_one_reinsert_validity();
        test_one_reinsert_bounds();
        test_objective_improvement();
        test_solution_consistency();
        std::cout << "\n✓ All tests passed\n";
    } catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
