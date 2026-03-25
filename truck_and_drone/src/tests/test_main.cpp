#include <cassert>
#include <iostream>
#include <vector>
#include "algorithms/gam_escape_algorithm.h"
#include "algorithms/greedy_drone_cover.h"
#include "algorithms/nearest_neighbour.h"
#include "algorithms/simple_initial_solution.h"
#include "datahandling/convert_to_submission.h"
#include "datahandling/datasets.h"
#include "datahandling/instance.h"
#include "datahandling/reader.h"
#include "operators/alns/alns_composite.h"
#include "operators/one_reinsert.h"
#include "operators/operator.h"
#include "operators/replace_drone_delivery.h"
#include "operators/three_opt.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"

using namespace datasets;

void test_instance_loading() {
    Instance instance = read_instance(datasets::contest);
    assert(!instance.truck_matrix.empty());
    assert(!instance.drone_matrix.empty());
    assert(instance.n > 0);
    std::cout << "test_instance_loading passed\n";
}

void test_nearest_neighbour_validity() {
    Instance instance = read_instance(datasets::contest);
    Solution solution = nearest_neighbour(instance);
    assert(master_check(instance, solution, true));
    std::cout << "test_nearest_neighbour_validity passed\n";
}

void test_greedy_drone_cover_validity() {
    Instance instance = read_instance(datasets::contest);
    Solution initial = nearest_neighbour(instance);
    Solution solution = greedy_drone_cover(instance, initial);
    assert(master_check(instance, solution, true));
    std::cout << "test_greedy_drone_cover_validity passed\n";
}

void test_one_reinsert_validity() {
    Instance instance = read_instance(datasets::contest);
    Solution initial = nearest_neighbour(instance);
    Solution drone = greedy_drone_cover(instance, initial);

    Solution edited = drone; // copy
    bool success = one_reinsert(instance, edited, 2, 2, 12);
    assert(success);
    assert(master_check(instance, edited, true));

    std::cout << "test_one_reinsert_validity passed\n";
}

void test_one_reinsert_bounds() {
    Instance instance = read_instance(datasets::contest);
    Solution initial = nearest_neighbour(instance);
    Solution drone = greedy_drone_cover(instance, initial);

    Solution edited = drone;
    bool success = one_reinsert(instance, edited, 1, 1, 5);
    assert(success);
    assert(edited.truck_route.size() <= drone.truck_route.size() + 1);

    std::cout << "test_one_reinsert_bounds passed\n";
}

void test_objective_improvement() {
    Instance instance = read_instance(datasets::contest);
    Solution initial = nearest_neighbour(instance);
    Solution drone = greedy_drone_cover(instance, initial);

    long long initial_cost = objective_function_impl(instance, drone);

    Solution edited = drone;
    bool success = one_reinsert(instance, edited, 2, 2, 12);
    assert(success);

    long long edited_cost = objective_function_impl(instance, edited);
    assert(edited_cost <= initial_cost * 1.01);

    std::cout << "test_objective_improvement passed\n";
}

void test_solution_consistency() {
    Instance instance = read_instance(datasets::contest);
    Solution initial = nearest_neighbour(instance);
    Solution drone = greedy_drone_cover(instance, initial);

    Solution edited = drone;
    bool success = one_reinsert(instance, edited, 2, 2, 12);
    assert(success);

    assert(!convert_to_submission(edited).empty());

    std::cout << "test_solution_consistency passed\n";
}

void test_three_opt_improves_truck_route() {
    Instance instance{};
    instance.n = 4;
    instance.m = 2;
    instance.lim = 1000;
    instance.truck_matrix = {
        {0, 1, 2, 3, 4},
        {1, 0, 1, 2, 3},
        {2, 1, 0, 1, 2},
        {3, 2, 1, 0, 1},
        {4, 3, 2, 1, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{{0, 1, 3, 2, 4}, {}};
    const long long initial_cost =
        objective_function_truck_only(instance, solution.truck_route);

    bool success = three_opt(instance, solution, 1, 2, 3);

    assert(success);
    assert(solution.truck_route == std::vector<int>({0, 1, 2, 3, 4}));
    assert(objective_function_truck_only(instance, solution.truck_route) < initial_cost);

    std::cout << "test_three_opt_improves_truck_route passed\n";
}

void test_two_opt_greedy_improves_truck_route() {
    Instance instance{};
    instance.n = 4;
    instance.m = 2;
    instance.lim = 1000;
    instance.truck_matrix = {
        {0, 1, 2, 3, 4},
        {1, 0, 1, 2, 3},
        {2, 1, 0, 1, 2},
        {3, 2, 1, 0, 1},
        {4, 3, 2, 1, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{{0, 1, 3, 2, 4}, {}};
    const long long initial_cost =
        objective_function_truck_only(instance, solution.truck_route);

    bool success = two_opt_greedy(instance, solution);

    assert(success);
    assert(solution.truck_route == std::vector<int>({0, 1, 2, 3, 4}));
    assert(objective_function_truck_only(instance, solution.truck_route) < initial_cost);

    std::cout << "test_two_opt_greedy_improves_truck_route passed\n";
}

void test_replace_drone_delivery_moves_customer_back_to_truck() {
    Instance instance{};
    instance.n = 4;
    instance.m = 2;
    instance.lim = 1000;
    instance.truck_matrix = {
        {0, 1, 2, 3, 4},
        {1, 0, 1, 2, 3},
        {2, 1, 0, 1, 2},
        {3, 2, 1, 0, 1},
        {4, 3, 2, 1, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{
        {0, 1, 3, 4},
        {
            DroneCollection{{1}, {2}, {2}},
            DroneCollection{},
        }};

    bool success = replace_drone_delivery(instance, solution, 0, 0);

    assert(success);
    assert(solution.drones[0].deliver_nodes.empty());
    assert(solution.truck_route == std::vector<int>({0, 1, 2, 3, 4}));
    assert(master_check(instance, solution, true));

    std::cout << "test_replace_drone_delivery_moves_customer_back_to_truck passed\n";
}

void test_replace_drone_delivery_greedy_moves_customer_back_to_truck() {
    Instance instance{};
    instance.n = 4;
    instance.m = 2;
    instance.lim = 1000;
    instance.truck_matrix = {
        {0, 1, 2, 3, 4},
        {1, 0, 1, 2, 3},
        {2, 1, 0, 1, 2},
        {3, 2, 1, 0, 1},
        {4, 3, 2, 1, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{
        {0, 1, 3, 4},
        {
            DroneCollection{{1}, {2}, {2}},
            DroneCollection{},
        }};

    bool success = replace_drone_delivery_greedy(instance, solution);

    assert(success);
    assert(solution.drones[0].deliver_nodes.empty());
    assert(solution.truck_route == std::vector<int>({0, 1, 2, 3, 4}));
    assert(master_check(instance, solution, true));

    std::cout << "test_replace_drone_delivery_greedy_moves_customer_back_to_truck passed\n";
}

void test_drone_demotion_shake_moves_customer_back_to_truck() {
    Instance instance{};
    instance.n = 4;
    instance.m = 2;
    instance.lim = 1000;
    instance.truck_matrix = {
        {0, 1, 2, 3, 4},
        {1, 0, 1, 2, 3},
        {2, 1, 0, 1, 2},
        {3, 2, 1, 0, 1},
        {4, 3, 2, 1, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{
        {0, 1, 3, 4},
        {
            DroneCollection{{1}, {2}, {2}},
            DroneCollection{},
        }};

    bool success = drone_demotion_shake(instance, solution);

    assert(success);
    assert(solution.drones[0].deliver_nodes.empty());
    assert(solution.truck_route == std::vector<int>({0, 1, 2, 3, 4}));
    assert(master_check(instance, solution, true));

    std::cout << "test_drone_demotion_shake_moves_customer_back_to_truck passed\n";
}

void test_alns_operator_applies_remove_then_insert() {
    Instance instance{};
    Solution solution{{0, 1}, {}};
    std::vector<int> call_order;

    ALNSOperator alns_op{
        [&](const Instance &, Solution &candidate, int n) {
            call_order.push_back(1);
            assert(n == 2);
            candidate.truck_route.push_back(2);
            return std::pair<bool, std::vector<int>>{true, std::vector<int>{7, 8}};
        },
        [&](const Instance &, Solution &candidate, std::vector<int> removed, int k) {
            call_order.push_back(2);
            assert((removed == std::vector<int>{7, 8}));
            assert(k == 3);
            candidate.truck_route.push_back(3);
            return true;
        },
        2,
        3};

    bool success = make_alns_operator(alns_op)(instance, solution);

    assert(success);
    assert((call_order == std::vector<int>{1, 2}));
    assert((solution.truck_route == std::vector<int>{0, 1, 2, 3}));

    std::cout << "test_alns_operator_applies_remove_then_insert passed\n";
}

void test_alns_operator_rolls_back_failed_insert() {
    Instance instance{};
    Solution solution{{0, 1, 2}, {}};
    Solution original = solution;

    ALNSOperator alns_op{
        [](const Instance &, Solution &candidate, int) {
            candidate.truck_route.pop_back();
            return std::pair<bool, std::vector<int>>{true, std::vector<int>{2}};
        },
        [](const Instance &, Solution &, std::vector<int>, int) {
            return false;
        }};

    bool success = make_alns_operator(alns_op)(instance, solution);

    assert(!success);
    assert(solution.truck_route == original.truck_route);

    std::cout << "test_alns_operator_rolls_back_failed_insert passed\n";
}

void test_alns_pair_combination_materializes_named_operators() {
    std::vector<NamedRemovalHeuristic> removals = {
        {"Random Removal", [](const Instance &, Solution &, int) {
             return std::pair<bool, std::vector<int>>{true, std::vector<int>{}};
         }},
        {"Worst Removal", [](const Instance &, Solution &, int) {
             return std::pair<bool, std::vector<int>>{true, std::vector<int>{}};
         }}};
    std::vector<NamedInsertionHeuristic> insertions = {
        {"Greedy Insert", [](const Instance &, Solution &, std::vector<int>, int) { return true; }}};

    std::vector<NamedOperator> combined = combine_alns_operator_pairs(removals, insertions, 4, 1);

    assert(combined.size() == 2);
    assert(combined[0].name == "Random Removal + Greedy Insert");
    assert(combined[1].name == "Worst Removal + Greedy Insert");

    std::cout << "test_alns_pair_combination_materializes_named_operators passed\n";
}

void test_adaptive_composite_operator_rolls_back_failed_insert() {
    Instance instance{};
    Solution solution{{0, 1, 2}, {}};
    Solution original = solution;

    AdaptiveCompositeOperator composite(
        {[](const Instance &, Solution &candidate, int) {
            candidate.truck_route.pop_back();
            return std::pair<bool, std::vector<int>>{true, std::vector<int>{2}};
        }},
        {[](const Instance &, Solution &, std::vector<int>, int) {
            return false;
        }},
        1);

    bool success = composite(instance, solution);

    assert(!success);
    assert(solution.truck_route == original.truck_route);

    std::cout << "test_adaptive_composite_operator_rolls_back_failed_insert passed\n";
}

void test_gam_escape_algorithm_returns_best_seen_solution() {
    Instance instance{};
    instance.n = 3;
    instance.m = 2;
    instance.lim = 1000;
    instance.truck_matrix = {
        {0, 1, 10, 10},
        {1, 0, 1, 1},
        {10, 1, 0, 5},
        {10, 1, 5, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution initial{{0, 1, 2, 3}, {DroneCollection{}, DroneCollection{}}};
    const long long initial_cost = objective_function_impl(instance, initial);

    std::vector<NamedOperator> ops = {
        {"Worsen route", [](const Instance &, Solution &sol) {
             std::swap(sol.truck_route[2], sol.truck_route[3]);
             return true;
         }}};
    std::vector<double> weights = {1.0};

    Solution escaped = gam_escape_algorithm(instance, initial, ops, weights, 3);

    assert(escaped.truck_route == initial.truck_route);
    assert(objective_function_impl(instance, escaped) == initial_cost);

    std::cout << "test_gam_escape_algorithm_returns_best_seen_solution passed\n";
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
        test_three_opt_improves_truck_route();
        test_two_opt_greedy_improves_truck_route();
        test_replace_drone_delivery_moves_customer_back_to_truck();
        test_replace_drone_delivery_greedy_moves_customer_back_to_truck();
        test_drone_demotion_shake_moves_customer_back_to_truck();
        test_alns_operator_applies_remove_then_insert();
        test_alns_operator_rolls_back_failed_insert();
        test_alns_pair_combination_materializes_named_operators();
        test_adaptive_composite_operator_rolls_back_failed_insert();
        test_gam_escape_algorithm_returns_best_seen_solution();
        std::cout << "\nAll tests passed\n";
    } catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
