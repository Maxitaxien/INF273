#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>
#include "algorithms/gam_escape_algorithm.h"
#include "algorithms/gam_solution_cache.h"
#include "algorithms/greedy_drone_cover.h"
#include "algorithms/nearest_neighbour.h"
#include "algorithms/simple_initial_solution.h"
#include "datahandling/convert_to_submission.h"
#include "datahandling/datasets.h"
#include "datahandling/instance.h"
#include "datahandling/instance_preprocessing.h"
#include "datahandling/reader.h"
#include "datahandling/save_to_csv.h"
#include "operators/alns/alns_composite.h"
#include "operators/customer_slot_helpers.h"
#include "operators/drone_rendezvous_shift.h"
#include "operators/exact_segment_reopt.h"
#include "operators/one_reinsert.h"
#include "operators/operator.h"
#include "operators/or_opt_segment_relocate.h"
#include "operators/helpers.h"
#include "operators/nearest_neighbour_reassign.h"
#include "operators/replace_drone_delivery.h"
#include "operators/replace_truck_delivery.h"
#include "solution_fixers/solution_fixers.h"
#include "tsp/linkernsolver.h"
#include "operators/two_opt.h"
#include "operators/three_opt.h"
#include "general/random.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"

using namespace datasets;

void test_instance_loading() {
    Instance instance = read_instance(datasets::contest);
    assert(!instance.truck_matrix.empty());
    assert(!instance.drone_matrix.empty());
    assert(!instance.pure_drone_feasible.empty());
    assert(instance.n > 0);
    std::cout << "test_instance_loading passed\n";
}

void test_pure_drone_feasibility_cache_matches_distance_limit() {
    Instance instance{};
    instance.n = 2;
    instance.m = 2;
    instance.lim = 7;
    instance.truck_matrix = {
        {0, 1, 1},
        {1, 0, 1},
        {1, 1, 0},
    };
    instance.drone_matrix = {
        {0, 4, 5},
        {4, 0, 3},
        {5, 3, 0},
    };

    precompute_pure_drone_feasibility(instance);

    assert(pure_drone_flight_within_limit(instance, 0, 1, 2));
    assert(!pure_drone_flight_within_limit(instance, 0, 2, 1));
    assert(!pure_drone_flight_within_limit(instance, -1, 1, 0));

    std::cout << "test_pure_drone_feasibility_cache_matches_distance_limit passed\n";
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
    instance.lim = 20000;
    instance.truck_matrix = {
        {0, 100, 200, 300, 400},
        {100, 0, 100, 200, 300},
        {200, 100, 0, 100, 200},
        {300, 200, 100, 0, 100},
        {400, 300, 200, 100, 0},
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
    instance.lim = 20000;
    instance.truck_matrix = {
        {0, 100, 200, 300, 400},
        {100, 0, 100, 200, 300},
        {200, 100, 0, 100, 200},
        {300, 200, 100, 0, 100},
        {400, 300, 200, 100, 0},
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

void test_two_opt_greedy_never_accepts_worse_full_objective() {
    Instance instance = read_instance(datasets::f20);
    Solution solution = greedy_drone_cover(instance, nearest_neighbour(instance));
    const long long initial_cost = objective_function_impl(instance, solution);

    Solution candidate = solution;
    const bool success = two_opt_greedy(instance, candidate);

    if (success) {
        assert(master_check(instance, candidate, true));
        assert(objective_function_impl(instance, candidate) < initial_cost);
    } else {
        assert(candidate.truck_route == solution.truck_route);
        assert(convert_to_submission(candidate) == convert_to_submission(solution));
    }

    std::cout << "test_two_opt_greedy_never_accepts_worse_full_objective passed\n";
}

void test_two_opt_first_improvement_improves_truck_route() {
    Instance instance{};
    instance.n = 4;
    instance.m = 2;
    instance.lim = 20000;
    instance.truck_matrix = {
        {0, 100, 200, 300, 400},
        {100, 0, 100, 200, 300},
        {200, 100, 0, 100, 200},
        {300, 200, 100, 0, 100},
        {400, 300, 200, 100, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{{0, 1, 3, 2, 4}, {}};
    const long long initial_cost =
        objective_function_truck_only(instance, solution.truck_route);

    const bool success = two_opt_first_improvement(instance, solution);

    assert(success);
    assert(solution.truck_route == std::vector<int>({0, 1, 2, 3, 4}));
    assert(objective_function_truck_only(instance, solution.truck_route) < initial_cost);

    std::cout << "test_two_opt_first_improvement_improves_truck_route passed\n";
}

void test_two_opt_arrival_screened_improves_truck_route() {
    Instance instance{};
    instance.n = 4;
    instance.m = 2;
    instance.lim = 20000;
    instance.truck_matrix = {
        {0, 100, 200, 300, 400},
        {100, 0, 100, 200, 300},
        {200, 100, 0, 100, 200},
        {300, 200, 100, 0, 100},
        {400, 300, 200, 100, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{{0, 1, 3, 2, 4}, {}};
    const long long initial_cost = objective_function_impl(instance, solution);

    const bool success = two_opt_arrival_screened(instance, solution);

    assert(success);
    assert(solution.truck_route == std::vector<int>({0, 1, 2, 3, 4}));
    assert(objective_function_impl(instance, solution) < initial_cost);

    std::cout << "test_two_opt_arrival_screened_improves_truck_route passed\n";
}

void test_two_opt_random_swaps_combined_customer_slots() {
    Instance instance{};
    instance.n = 2;
    instance.m = 2;
    instance.lim = 20000;
    instance.truck_matrix = {
        {0, 100, 400},
        {100, 0, 100},
        {100, 300, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{{0, 2, 1}, {}};
    const long long initial_cost = objective_function_impl(instance, solution);

    bool success = two_opt_random(instance, solution);

    assert(success);
    assert(solution.truck_route == std::vector<int>({0, 1, 2}));
    assert(master_check(instance, solution, true));
    assert(objective_function_impl(instance, solution) < initial_cost);

    std::cout << "test_two_opt_random_swaps_combined_customer_slots passed\n";
}

void test_linkern_solver_solves_current_truck_subset() {
    Instance instance{};
    instance.n = 4;
    instance.m = 2;
    instance.lim = 20000;
    instance.truck_matrix = {
        {0, 100, 200, 300, 400},
        {100, 0, 100, 200, 300},
        {200, 100, 0, 100, 200},
        {300, 200, 100, 0, 100},
        {400, 300, 200, 100, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    LinkernSolver solver(instance.truck_matrix);
    const std::vector<int> initial_route = {0, 1, 3, 2, 4};
    const double initial_length = solver.tour_length(initial_route);
    const LinkernTour optimized = solver.solve_route(initial_route, 0);

    assert(optimized.tour.size() == initial_route.size());
    assert(optimized.tour.front() == 0);
    assert(optimized.value < initial_length);
    assert(std::abs(optimized.value - 800.0) < 1e-9);

    std::vector<int> sorted = optimized.tour;
    std::sort(sorted.begin(), sorted.end());
    assert(sorted == std::vector<int>({0, 1, 2, 3, 4}));

    std::cout << "test_linkern_solver_solves_current_truck_subset passed\n";
}

void test_concorde_linkern_operator_improves_truck_route() {
    Instance instance{};
    instance.n = 4;
    instance.m = 2;
    instance.lim = 20000;
    instance.truck_matrix = {
        {0, 100, 200, 300, 400},
        {100, 0, 100, 200, 300},
        {200, 100, 0, 100, 200},
        {300, 200, 100, 0, 100},
        {400, 300, 200, 100, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{{0, 1, 3, 2, 4}, {}};
    const long long initial_cost = objective_function_impl(instance, solution);

    const bool success = concorde_linkern_improve(instance, solution);

    assert(success);
    assert(master_check(instance, solution, true));
    assert(objective_function_impl(instance, solution) < initial_cost);

    LinkernSolver solver(instance.truck_matrix);
    assert(std::abs(solver.tour_length(solution.truck_route) - 800.0) < 1e-9);

    std::cout << "test_concorde_linkern_operator_improves_truck_route passed\n";
}

void test_sample_contiguous_slot_indices_returns_block() {
    gen.seed(42);

    const std::vector<int> indices = sample_contiguous_slot_indices(8, 3);

    assert(indices.size() == 3);
    assert(indices[1] == indices[0] + 1);
    assert(indices[2] == indices[1] + 1);
    assert(indices.front() >= 0);
    assert(indices.back() < 8);

    std::cout << "test_sample_contiguous_slot_indices_returns_block passed\n";
}

void test_three_opt_random_permutates_combined_customer_slots() {
    Instance instance{};
    instance.n = 3;
    instance.m = 2;
    instance.lim = 20000;
    instance.truck_matrix = {
        {0, 100, 300, 400},
        {100, 0, 100, 300},
        {100, 100, 0, 100},
        {400, 300, 100, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{{0, 1, 3, 2}, {}};
    const long long initial_cost = objective_function_impl(instance, solution);

    bool success = three_opt_random(instance, solution);

    assert(success);
    assert(solution.truck_route == std::vector<int>({0, 1, 2, 3}));
    assert(master_check(instance, solution, true));
    assert(objective_function_impl(instance, solution) < initial_cost);

    std::cout << "test_three_opt_random_permutates_combined_customer_slots passed\n";
}

void test_or_opt_segment_relocate_moves_block() {
    Instance instance{};
    instance.n = 4;
    instance.m = 2;
    instance.lim = 20000;
    instance.truck_matrix = {
        {0, 100, 200, 300, 400},
        {100, 0, 100, 200, 300},
        {200, 100, 0, 100, 200},
        {300, 200, 100, 0, 100},
        {400, 300, 200, 100, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{{0, 1, 3, 2, 4}, {}};

    const bool success = or_opt_segment_relocate(instance, solution, 1, 1, 2);

    assert(success);
    assert(solution.truck_route == std::vector<int>({0, 1, 2, 3, 4}));
    assert(master_check(instance, solution, true));

    std::cout << "test_or_opt_segment_relocate_moves_block passed\n";
}

void test_or_opt_segment_relocate_first_improvement_improves_route() {
    Instance instance{};
    instance.n = 4;
    instance.m = 2;
    instance.lim = 20000;
    instance.truck_matrix = {
        {0, 100, 200, 300, 400},
        {100, 0, 100, 200, 300},
        {200, 100, 0, 100, 200},
        {300, 200, 100, 0, 100},
        {400, 300, 200, 100, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{{0, 1, 3, 2, 4}, {}};
    const long long initial_cost = objective_function_impl(instance, solution);

    const bool success = or_opt_segment_relocate_first_improvement(instance, solution);

    assert(success);
    assert(master_check(instance, solution, true));
    assert(objective_function_impl(instance, solution) < initial_cost);

    std::cout << "test_or_opt_segment_relocate_first_improvement_improves_route passed\n";
}

void test_or_opt_segment_relocate_remaps_drone_anchor_indices() {
    Instance instance{};
    instance.n = 5;
    instance.m = 2;
    instance.lim = 300;
    instance.truck_matrix = {
        {0, 100, 200, 300, 400, 500},
        {100, 0, 100, 200, 300, 400},
        {200, 100, 0, 100, 200, 300},
        {300, 200, 100, 0, 100, 200},
        {400, 300, 200, 100, 0, 100},
        {500, 400, 300, 200, 100, 0},
    };
    instance.drone_matrix = {
        {0, 100, 200, 200, 400, 500},
        {100, 0, 100, 300, 300, 100},
        {200, 100, 0, 500, 200, 100},
        {200, 300, 500, 0, 100, 500},
        {400, 300, 200, 100, 0, 300},
        {500, 100, 100, 500, 300, 0},
    };

    Solution solution{
        {0, 1, 3, 2, 4},
        {
            DroneCollection{{1}, {5}, {3}},
            DroneCollection{},
        }};

    assert(master_check(instance, solution, true));

    const bool success = or_opt_segment_relocate(instance, solution, 2, 1, 0);

    assert(success);
    assert(solution.truck_route == std::vector<int>({0, 1, 2, 3, 4}));
    assert(solution.drones[0].launch_indices == std::vector<int>({1}));
    assert(solution.drones[0].land_indices == std::vector<int>({2}));
    assert(master_check(instance, solution, true));

    std::cout << "test_or_opt_segment_relocate_remaps_drone_anchor_indices passed\n";
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

void test_replace_drone_delivery_picks_best_objective_insertion() {
    Instance instance{};
    instance.n = 4;
    instance.m = 2;
    instance.lim = 1000;
    instance.truck_matrix = {
        {0, 1, 6, 2, 3},
        {1, 0, 5, 1, 2},
        {6, 5, 0, 4, 1},
        {2, 1, 4, 0, 1},
        {3, 2, 1, 1, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{
        {0, 1, 3, 4},
        {
            DroneCollection{{1}, {2}, {2}},
            DroneCollection{},
        }};

    assert(master_check(instance, solution, true));

    const int insert_start = 2;
    const int insert_end = 3;
    long long brute_best = std::numeric_limits<long long>::max();

    for (int insert_idx = insert_start; insert_idx <= insert_end; ++insert_idx) {
        Solution candidate = solution;
        remove_drone_flight(candidate, 0, 0);
        candidate.truck_route.insert(candidate.truck_route.begin() + insert_idx, 2);
        if (!master_check(instance, candidate, false)) {
            continue;
        }

        brute_best = std::min(brute_best, objective_function_impl(instance, candidate));
    }

    assert(brute_best < std::numeric_limits<long long>::max());

    bool success = replace_drone_delivery(instance, solution, 0, 0);

    assert(success);
    assert(solution.drones[0].deliver_nodes.empty());
    assert(master_check(instance, solution, true));
    assert(objective_function_impl(instance, solution) == brute_best);

    std::cout << "test_replace_drone_delivery_picks_best_objective_insertion passed\n";
}

void test_nearest_neighbour_reassign_repairs_swapped_anchor_flights() {
    Instance instance{};
    instance.n = 5;
    instance.m = 2;
    instance.lim = 220;
    instance.truck_matrix = {
        {0, 100, 400, 400, 200, 500},
        {100, 0, 100, 400, 50, 400},
        {400, 100, 0, 400, 50, 300},
        {400, 400, 400, 0, 400, 400},
        {200, 50, 50, 400, 0, 100},
        {500, 400, 300, 400, 100, 0},
    };
    instance.drone_matrix = {
        {0, 100, 400, 70, 100, 400},
        {100, 0, 400, 80, 400, 500},
        {400, 400, 0, 80, 400, 500},
        {70, 80, 80, 0, 70, 500},
        {100, 400, 400, 400, 0, 100},
        {400, 500, 500, 500, 100, 0},
    };

    Solution solution{
        {0, 1, 2, 4, 5},
        {
            DroneCollection{{1}, {3}, {2}},
            DroneCollection{},
        }};

    assert(master_check(instance, solution, true));

    Solution broken_candidate = solution;
    std::iter_swap(
        broken_candidate.truck_route.begin() + 1,
        broken_candidate.truck_route.begin() + 3);
    remap_drone_anchor_indices_by_node(solution, broken_candidate);
    assert(!master_check(instance, broken_candidate, false));

    const bool success = nearest_neighbour_reassign(instance, solution, 1);

    assert(success);
    assert(solution.truck_route == std::vector<int>({0, 4, 2, 1, 5}));
    assert(solution.drones[0].deliver_nodes == std::vector<int>({3}));
    assert(master_check(instance, solution, true));

    std::cout << "test_nearest_neighbour_reassign_repairs_swapped_anchor_flights passed\n";
}

void test_replace_truck_delivery_repairs_removed_anchor_flights() {
    Instance instance{};
    instance.n = 5;
    instance.m = 2;
    instance.lim = 220;
    instance.truck_matrix = {
        {0, 100, 100, 400, 100, 100},
        {100, 0, 100, 400, 100, 400},
        {100, 100, 0, 400, 100, 400},
        {400, 400, 400, 0, 400, 400},
        {100, 100, 100, 400, 0, 100},
        {100, 400, 400, 400, 100, 0},
    };
    instance.drone_matrix = {
        {0, 100, 100, 200, 100, 100},
        {100, 0, 100, 70, 90, 400},
        {100, 100, 0, 80, 90, 400},
        {200, 100, 80, 0, 70, 400},
        {100, 90, 90, 70, 0, 100},
        {100, 400, 400, 400, 100, 0},
    };

    Solution solution{
        {0, 1, 2, 4, 5},
        {
            DroneCollection{},
            DroneCollection{{2}, {3}, {3}},
        }};

    assert(master_check(instance, solution, true));

    Solution broken_candidate = solution;
    pop_truck_delivery(broken_candidate, 2);
    remap_drone_anchor_indices_by_node(solution, broken_candidate);
    assert(!master_check(instance, broken_candidate, false));

    const bool success = replace_truck_delivery(instance, solution, 2, 0);

    assert(success);
    assert(solution.truck_route == std::vector<int>({0, 1, 4, 5}));
    assert(solution.drones[0].deliver_nodes == std::vector<int>({2}));
    assert(solution.drones[1].deliver_nodes == std::vector<int>({3}));
    assert(master_check(instance, solution, true));

    std::cout << "test_replace_truck_delivery_repairs_removed_anchor_flights passed\n";
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

void test_gam_escape_algorithm_tracks_best_seen_while_returning_endpoint() {
    Instance instance{};
    instance.n = 3;
    instance.m = 2;
    instance.lim = 1000;
    instance.truck_matrix = {
        {0, 100, 1000, 1000},
        {100, 0, 500, 100},
        {1000, 500, 0, 100},
        {1000, 100, 100, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution initial{{0, 1, 2, 3}, {DroneCollection{}, DroneCollection{}}};
    const long long initial_cost = objective_function_impl(instance, initial);

    std::vector<NamedOperator> ops = {
        {"Walk route", [step = 0](const Instance &, Solution &sol) mutable {
             if (step == 0)
             {
                 sol.truck_route = {0, 1, 3, 2};
             }
             else
             {
                 sol.truck_route = {0, 3, 1, 2};
             }
             ++step;
             return true;
         }}};
    std::vector<double> weights = {1.0};

    const GAMEscapeResult escaped =
        gam_escape_algorithm(instance, initial, ops, weights, 2);

    assert(escaped.incumbent.truck_route == std::vector<int>({0, 3, 1, 2}));
    assert(escaped.best_seen.truck_route == std::vector<int>({0, 1, 3, 2}));
    assert(escaped.found_new_best);
    assert(objective_function_impl(instance, escaped.best_seen) < initial_cost);
    assert(
        objective_function_impl(instance, escaped.incumbent) >
        objective_function_impl(instance, escaped.best_seen));

    std::cout << "test_gam_escape_algorithm_tracks_best_seen_while_returning_endpoint passed\n";
}

void test_gam_solution_cache_marks_first_visit_and_reuses_evaluation() {
    Instance instance{};
    instance.n = 3;
    instance.m = 2;
    instance.lim = 1000;
    instance.truck_matrix = {
        {0, 100, 1000, 1000},
        {100, 0, 500, 100},
        {1000, 500, 0, 100},
        {1000, 100, 100, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{{0, 1, 3, 2}, {DroneCollection{}, DroneCollection{}}};
    GAMSolutionCache cache;

    const GAMSolutionEvaluation first =
        evaluate_solution_with_cache(
            instance,
            solution,
            &cache,
            GAMFeasibilityMode::AssumeFeasible);
    const GAMSolutionEvaluation second =
        evaluate_solution_with_cache(
            instance,
            solution,
            &cache,
            GAMFeasibilityMode::AssumeFeasible);

    assert(first.is_new_solution);
    assert(first.feasible);
    assert(first.objective_known);
    assert(!second.is_new_solution);
    assert(second.feasible);
    assert(second.objective_known);
    assert(first.objective == second.objective);
    assert(cache.size() == 1);

    std::cout << "test_gam_solution_cache_marks_first_visit_and_reuses_evaluation passed\n";
}

void test_gam_solution_cache_verify_with_master_check_rejects_infeasible_solution() {
    Instance instance{};
    instance.n = 2;
    instance.m = 1;
    instance.lim = 1000;
    instance.truck_matrix = {
        {0, 10, 20},
        {10, 0, 10},
        {20, 10, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution infeasible{
        {0, 1, 2},
        {
            DroneCollection{{0}, {1}, {2}},
        }};
    assert(!master_check(instance, infeasible, false));

    GAMSolutionCache cache;
    const GAMSolutionEvaluation first =
        evaluate_solution_with_cache(
            instance,
            infeasible,
            &cache,
            GAMFeasibilityMode::VerifyWithMasterCheck);
    const GAMSolutionEvaluation second =
        evaluate_solution_with_cache(
            instance,
            infeasible,
            &cache,
            GAMFeasibilityMode::VerifyWithMasterCheck);

    assert(first.is_new_solution);
    assert(!first.feasible);
    assert(!first.objective_known);
    assert(!second.is_new_solution);
    assert(!second.feasible);
    assert(!second.objective_known);
    assert(cache.size() == 1);

    std::cout << "test_gam_solution_cache_verify_with_master_check_rejects_infeasible_solution passed\n";
}

void test_shaw_removal_greedy_repair_random_medium_preserves_feasibility_on_success() {
    Instance instance = read_instance(datasets::f10);
    Solution solution = greedy_drone_cover(instance, nearest_neighbour(instance));
    assert(master_check(instance, solution, false));

    const bool success = shaw_removal_greedy_repair_random_medium(instance, solution);
    if (success)
    {
        assert(master_check(instance, solution, false));
    }

    std::cout << "test_shaw_removal_greedy_repair_random_medium_preserves_feasibility_on_success passed\n";
}

void test_exchange_k_large_preserves_feasibility_on_success() {
    Instance instance = read_instance(datasets::f10);
    Solution solution = greedy_drone_cover(instance, nearest_neighbour(instance));
    assert(master_check(instance, solution, false));

    const bool success = exchange_k_large(instance, solution);
    if (success)
    {
        assert(master_check(instance, solution, false));
    }

    std::cout << "test_exchange_k_large_preserves_feasibility_on_success passed\n";
}

void test_two_opt_repairs_drone_schedule_after_route_reversal() {
    Instance instance{};
    instance.n = 5;
    instance.m = 2;
    instance.lim = 5;
    instance.truck_matrix = {
        {0, 1, 2, 3, 10, 11},
        {1, 0, 1, 2, 9, 10},
        {2, 1, 0, 1, 8, 9},
        {3, 2, 1, 0, 7, 8},
        {10, 9, 8, 7, 0, 1},
        {11, 10, 9, 8, 1, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{
        {0, 1, 3, 4, 5},
        {
            DroneCollection{{1}, {2}, {2}},
            DroneCollection{},
        }};

    assert(master_check(instance, solution, true));

    bool success = two_opt(instance, solution, 1, 3);

    assert(success);
    assert(solution.truck_route == std::vector<int>({0, 1, 4, 3, 5}));
    assert(master_check(instance, solution, true));
    assert(solution.drones[0].deliver_nodes == std::vector<int>({2}));
    assert(solution.truck_route.end() == std::find(solution.truck_route.begin(), solution.truck_route.end(), 2));

    std::cout << "test_two_opt_repairs_drone_schedule_after_route_reversal passed\n";
}

void test_collect_two_opt_affected_drone_flights_returns_intersections() {
    Solution solution{
        {0, 1, 3, 4, 6, 7, 8},
        {
            DroneCollection{{1, 4}, {2, 5}, {3, 5}},
            DroneCollection{},
        }};

    const std::vector<AffectedDroneFlight> affected =
        collect_two_opt_affected_drone_flights(solution, 1, 3);

    assert(affected.size() == 1);
    assert(affected[0].drone == 0);
    assert(affected[0].flight_idx == 0);
    assert(affected[0].delivery == 2);
    assert(affected[0].launch_idx == 1);
    assert(affected[0].land_idx == 3);

    std::cout << "test_collect_two_opt_affected_drone_flights_returns_intersections passed\n";
}

void test_repair_after_two_opt_localized_repairs_candidate() {
    Instance instance{};
    instance.n = 5;
    instance.m = 2;
    instance.lim = 5;
    instance.truck_matrix = {
        {0, 1, 2, 3, 10, 11},
        {1, 0, 1, 2, 9, 10},
        {2, 1, 0, 1, 8, 9},
        {3, 2, 1, 0, 7, 8},
        {10, 9, 8, 7, 0, 1},
        {11, 10, 9, 8, 1, 0},
    };
    instance.drone_matrix = instance.truck_matrix;
    instance.drone_matrix[2][0] = 10;

    Solution before_move{
        {0, 1, 3, 4, 5},
        {
            DroneCollection{{1}, {2}, {2}},
            DroneCollection{},
        }};

    Solution candidate = before_move;
    std::reverse(
        candidate.truck_route.begin() + 2,
        candidate.truck_route.begin() + 4);

    assert(!master_check(instance, candidate, false));
    assert(repair_after_two_opt_localized(instance, before_move, 1, 3, candidate));
    assert(master_check(instance, candidate, true));
    assert(candidate.truck_route == std::vector<int>({0, 1, 4, 3, 5}));
    assert(candidate.drones[0].deliver_nodes == std::vector<int>({2}));

    std::cout << "test_repair_after_two_opt_localized_repairs_candidate passed\n";
}

void test_solution_visualization_writes_jpg() {
    Instance instance{};
    instance.n = 3;
    instance.m = 2;
    instance.lim = 10000;

    const std::vector<std::pair<double, double>> coordinates = {
        {0.0, 0.0},
        {-1000.0, 2000.0},
        {1000.0, 2000.0},
        {0.0, 4000.0},
    };

    auto make_distance_matrix = [&](const std::vector<std::pair<double, double>> &points) {
        std::vector<std::vector<long long>> matrix(points.size(), std::vector<long long>(points.size(), 0));
        for (int i = 0; i < (int)(points.size()); ++i) {
            for (int j = 0; j < (int)(points.size()); ++j) {
                const double dx = points[i].first - points[j].first;
                const double dy = points[i].second - points[j].second;
                matrix[i][j] = (long long)(std::llround(std::sqrt(dx * dx + dy * dy)));
            }
        }
        return matrix;
    };

    instance.truck_matrix = make_distance_matrix(coordinates);
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{
        {0, 1, 3},
        {
            DroneCollection{{1}, {2}, {2}},
            DroneCollection{},
        }};

    assert(master_check(instance, solution, true));

    const std::filesystem::path output =
        std::filesystem::temp_directory_path() / "truck_and_drone_solution_visualization_test.jpg";
    std::filesystem::remove(output);

    const bool success = solution.save_visualization(instance, output.string());

    assert(success);
    assert(std::filesystem::exists(output));
    assert(std::filesystem::file_size(output) > 0);

    std::filesystem::remove(output);

    std::cout << "test_solution_visualization_writes_jpg passed\n";
}

void test_drone_rendezvous_shift_moves_flight_within_window() {
    Instance instance{};
    instance.n = 4;
    instance.m = 2;
    instance.lim = 20000;
    instance.truck_matrix = {
        {0, 100, 100, 10100, 10200},
        {100, 0, 100, 10000, 10100},
        {100, 100, 0, 100, 200},
        {10100, 10000, 100, 0, 100},
        {10200, 10100, 200, 100, 0},
    };
    instance.drone_matrix = {
        {0, 100, 10000, 10100, 10200},
        {100, 0, 100, 10100, 100},
        {10000, 100, 0, 100, 100},
        {10100, 10100, 100, 0, 100},
        {10200, 100, 100, 100, 0},
    };

    Solution solution{
        {0, 1, 3, 4},
        {
            DroneCollection{{0}, {2}, {2}},
            DroneCollection{},
        }};

    assert(master_check(instance, solution, true));
    const long long initial_cost = objective_function_impl(instance, solution);

    const bool success = drone_rendezvous_shift(instance, solution, 0, 0, 2, 2);

    assert(success);
    assert(solution.drones[0].launch_indices == std::vector<int>({1}));
    assert(solution.drones[0].land_indices[0] > 1);
    assert(master_check(instance, solution, true));
    assert(objective_function_impl(instance, solution) < initial_cost);

    std::cout << "test_drone_rendezvous_shift_moves_flight_within_window passed\n";
}

void test_drone_rendezvous_shift_first_improvement_applies_move() {
    Instance instance{};
    instance.n = 4;
    instance.m = 2;
    instance.lim = 20000;
    instance.truck_matrix = {
        {0, 100, 100, 10100, 10200},
        {100, 0, 100, 10000, 10100},
        {100, 100, 0, 100, 200},
        {10100, 10000, 100, 0, 100},
        {10200, 10100, 200, 100, 0},
    };
    instance.drone_matrix = {
        {0, 100, 10000, 10100, 10200},
        {100, 0, 100, 10100, 100},
        {10000, 100, 0, 100, 100},
        {10100, 10100, 100, 0, 100},
        {10200, 100, 100, 100, 0},
    };

    Solution solution{
        {0, 1, 3, 4},
        {
            DroneCollection{{0}, {2}, {2}},
            DroneCollection{},
        }};

    assert(master_check(instance, solution, true));
    const long long initial_cost = objective_function_impl(instance, solution);

    const bool success = drone_rendezvous_shift_first_improvement(instance, solution);

    assert(success);
    assert(solution.drones[0].launch_indices == std::vector<int>({1}));
    assert(solution.drones[0].land_indices[0] > 1);
    assert(master_check(instance, solution, true));
    assert(objective_function_impl(instance, solution) < initial_cost);

    std::cout << "test_drone_rendezvous_shift_first_improvement_applies_move passed\n";
}

void test_drone_rendezvous_shift_best_improvement_picks_best_flight() {
    Instance instance{};
    instance.n = 7;
    instance.m = 2;
    instance.lim = 20000;

    instance.truck_matrix.assign(8, std::vector<long long>(8, 10000));
    instance.drone_matrix.assign(8, std::vector<long long>(8, 10000));
    for (int i = 0; i < 8; ++i)
    {
        instance.truck_matrix[i][i] = 0;
        instance.drone_matrix[i][i] = 0;
    }

    const std::vector<int> route = {0, 1, 3, 4, 6, 7};
    for (int idx = 1; idx < (int)route.size(); ++idx)
    {
        const int prev = route[idx - 1];
        const int curr = route[idx];
        instance.truck_matrix[prev][curr] = 100;
        instance.truck_matrix[curr][prev] = 100;
    }

    instance.drone_matrix[0][2] = 10000;
    instance.drone_matrix[1][2] = 100;
    instance.drone_matrix[3][2] = 150;
    instance.drone_matrix[4][2] = 250;
    instance.drone_matrix[2][3] = 100;
    instance.drone_matrix[2][4] = 200;
    instance.drone_matrix[2][6] = 300;
    instance.drone_matrix[2][7] = 400;

    instance.drone_matrix[1][5] = 650;
    instance.drone_matrix[3][5] = 450;
    instance.drone_matrix[4][5] = 500;
    instance.drone_matrix[6][5] = 100;
    instance.drone_matrix[5][4] = 120;
    instance.drone_matrix[5][6] = 100;
    instance.drone_matrix[5][7] = 100;

    Solution solution{
        route,
        {
            DroneCollection{{0}, {2}, {2}},
            DroneCollection{{3}, {5}, {5}},
        }};

    assert(master_check(instance, solution, true));
    const long long initial_cost = objective_function_impl(instance, solution);

    const int best_improvement_window = 3;
    int improving_shifts = 0;
    long long best_cost = std::numeric_limits<long long>::max();

    for (int drone = 0; drone < (int)(solution.drones.size()); ++drone)
    {
        const int flight_count = (int)(solution.drones[drone].deliver_nodes.size());
        for (int flight_idx = 0; flight_idx < flight_count; ++flight_idx)
        {
            Solution candidate = solution;
            if (!drone_rendezvous_shift(
                    instance,
                    candidate,
                    drone,
                    flight_idx,
                    best_improvement_window,
                    best_improvement_window))
            {
                continue;
            }

            const long long candidate_cost = objective_function_impl(instance, candidate);
            if (candidate_cost < initial_cost)
            {
                ++improving_shifts;
                best_cost = std::min(best_cost, candidate_cost);
            }
        }
    }

    assert(improving_shifts >= 2);

    const bool success = drone_rendezvous_shift_best_improvement(instance, solution);

    assert(success);
    assert(master_check(instance, solution, true));
    assert(objective_function_impl(instance, solution) <= best_cost);
    assert(best_cost < initial_cost);

    std::cout << "test_drone_rendezvous_shift_best_improvement_picks_best_flight passed\n";
}

void test_drone_rendezvous_shift_best_improvement_returns_best_feasible_move() {
    Instance instance{};
    instance.n = 4;
    instance.m = 2;
    instance.lim = 1000;
    instance.truck_matrix = {
        {0, 100, 100, 100, 100},
        {100, 0, 100, 100, 100},
        {100, 100, 0, 100, 100},
        {100, 100, 100, 0, 100},
        {100, 100, 100, 100, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{
        {0, 1, 3, 4},
        {
            DroneCollection{{0}, {2}, {2}},
            DroneCollection{},
        }};

    assert(master_check(instance, solution, true));
    const long long initial_cost = objective_function_impl(instance, solution);

    const bool success = drone_rendezvous_shift_best_improvement(instance, solution);

    assert(success);
    assert(master_check(instance, solution, true));
    assert(objective_function_impl(instance, solution) == initial_cost);
    assert(solution.drones[0].launch_indices != std::vector<int>({0}) ||
           solution.drones[0].land_indices != std::vector<int>({2}));

    std::cout << "test_drone_rendezvous_shift_best_improvement_returns_best_feasible_move passed\n";
}

void test_single_drone_planner_shake_returns_false_without_drone_customers() {
    Instance instance{};
    instance.n = 3;
    instance.m = 2;
    instance.lim = 1000;
    instance.truck_matrix = {
        {0, 100, 200, 300},
        {100, 0, 100, 200},
        {200, 100, 0, 100},
        {300, 200, 100, 0},
    };
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{
        {0, 1, 2, 3},
        {
            DroneCollection{},
            DroneCollection{},
        }};

    assert(master_check(instance, solution, true));
    assert(!single_drone_planner_shake(instance, solution));

    std::cout << "test_single_drone_planner_shake_returns_false_without_drone_customers passed\n";
}

void test_single_drone_planner_shake_changes_one_drone_schedule() {
    gen.seed(42);

    Instance instance{};
    instance.n = 4;
    instance.m = 2;
    instance.lim = 20000;
    instance.truck_matrix = {
        {0, 100, 100, 10100, 10200},
        {100, 0, 100, 10000, 10100},
        {100, 100, 0, 100, 200},
        {10100, 10000, 100, 0, 100},
        {10200, 10100, 200, 100, 0},
    };
    instance.drone_matrix = {
        {0, 100, 10000, 10100, 10200},
        {100, 0, 100, 10100, 100},
        {10000, 100, 0, 100, 100},
        {10100, 10100, 100, 0, 100},
        {10200, 100, 100, 100, 0},
    };

    Solution solution{
        {0, 1, 3, 4},
        {
            DroneCollection{{0}, {2}, {2}},
            DroneCollection{},
        }};

    assert(master_check(instance, solution, true));
    const DroneCollection original = solution.drones[0];

    const bool success = single_drone_planner_shake(instance, solution);

    assert(success);
    assert(master_check(instance, solution, true));
    assert(
        solution.drones[0].launch_indices != original.launch_indices ||
        solution.drones[0].land_indices != original.land_indices);

    std::cout << "test_single_drone_planner_shake_changes_one_drone_schedule passed\n";
}

void test_replace_drone_delivery_targeted_moves_customer_to_truck() {
    Instance instance{};
    instance.n = 4;
    instance.m = 2;
    instance.lim = 20000;
    instance.truck_matrix = {
        {0, 100, 100, 10100, 10200},
        {100, 0, 100, 10000, 10100},
        {100, 100, 0, 100, 200},
        {10100, 10000, 100, 0, 100},
        {10200, 10100, 200, 100, 0},
    };
    instance.drone_matrix = {
        {0, 100, 10000, 10100, 10200},
        {100, 0, 100, 10100, 100},
        {10000, 100, 0, 100, 100},
        {10100, 10100, 100, 0, 100},
        {10200, 100, 100, 100, 0},
    };

    Solution solution{
        {0, 1, 3, 4},
        {
            DroneCollection{{0}, {2}, {1}},
            DroneCollection{},
        }};

    assert(master_check(instance, solution, true));

    const bool success = replace_drone_delivery_targeted(instance, solution);

    assert(success);
    assert(master_check(instance, solution, true));
    assert(solution.drones[0].deliver_nodes.empty());
    assert(std::find(solution.truck_route.begin(), solution.truck_route.end(), 2) != solution.truck_route.end());

    std::cout << "test_replace_drone_delivery_targeted_moves_customer_to_truck passed\n";
}

void test_exact_segment_reopt_uses_precomputed_pure_drone_cache() {
    Instance instance{};
    instance.n = 2;
    instance.m = 1;
    instance.lim = 100;
    instance.truck_matrix = {
        {0, 100, 400},
        {100, 0, 100},
        {400, 100, 0},
    };
    instance.drone_matrix = {
        {0, 200, 50},
        {100, 0, 200},
        {500, 50, 0},
    };
    precompute_pure_drone_feasibility(instance);

    Solution solution{{0, 2, 1}, {}};
    assert(master_check(instance, solution, true));
    const long long initial_cost = objective_function_impl(instance, solution);

    const bool success = exact_segment_reopt(instance, solution, 1, 1);

    assert(success);
    assert(master_check(instance, solution, true));
    assert(solution.truck_route == std::vector<int>({0, 1}));
    assert(!solution.drones.empty());
    assert(solution.drones[0].deliver_nodes == std::vector<int>({2}));
    assert(objective_function_impl(instance, solution) < initial_cost);

    std::cout << "test_exact_segment_reopt_uses_precomputed_pure_drone_cache passed\n";
}

void test_exact_segment_reopt_handles_suffix_window() {
    Instance instance{};
    instance.n = 3;
    instance.m = 0;
    instance.lim = 1000;
    instance.truck_matrix.assign(4, std::vector<long long>(4, 0));
    for (int i = 0; i <= instance.n; ++i) {
        for (int j = 0; j <= instance.n; ++j) {
            instance.truck_matrix[i][j] = 100LL * std::abs(i - j);
        }
    }
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{{0, 1, 3, 2}, {}};
    assert(master_check(instance, solution, true));
    const long long initial_cost = objective_function_impl(instance, solution);

    const bool success = exact_segment_reopt(instance, solution, 2, 2);

    assert(success);
    assert(master_check(instance, solution, true));
    assert(solution.truck_route == std::vector<int>({0, 1, 2, 3}));
    assert(objective_function_impl(instance, solution) < initial_cost);

    std::cout << "test_exact_segment_reopt_handles_suffix_window passed\n";
}

void test_exact_segment_reopt_random_small_retries_other_windows() {
    Instance instance{};
    instance.n = 5;
    instance.m = 0;
    instance.lim = 1000;
    instance.truck_matrix.assign(6, std::vector<long long>(6, 0));
    for (int i = 0; i <= instance.n; ++i) {
        for (int j = 0; j <= instance.n; ++j) {
            instance.truck_matrix[i][j] = 100LL * std::abs(i - j);
        }
    }
    instance.drone_matrix = instance.truck_matrix;

    Solution solution{{0, 2, 1, 3, 4, 5}, {}};
    assert(master_check(instance, solution, true));
    const long long initial_cost = objective_function_impl(instance, solution);

    gen.seed(42);
    const bool success = exact_segment_reopt_random_small(instance, solution);

    assert(success);
    assert(master_check(instance, solution, true));
    assert(solution.truck_route == std::vector<int>({0, 1, 2, 3, 4, 5}));
    assert(objective_function_impl(instance, solution) < initial_cost);

    std::cout << "test_exact_segment_reopt_random_small_retries_other_windows passed\n";
}

void test_save_gam_statistics_writes_operator_runtime_outputs() {
    const std::filesystem::path run_dir =
        std::filesystem::temp_directory_path() / "truck_and_drone_gam_statistics_test";
    std::filesystem::remove_all(run_dir);
    std::filesystem::create_directories(run_dir);

    GAMRunStatistics statistics;
    statistics.max_iterations = 100;
    statistics.segment_length = 10;
    statistics.stopping_condition = 5;
    statistics.best_found_iteration = 17;
    statistics.operator_names = {"Two-Opt Greedy"};
    statistics.iteration_stats.push_back(GAMIterationStatistics{
        1,
        0,
        -42,
        3.5,
        -1.0,
        7.25,
        true,
    });
    statistics.operator_stats.push_back(GAMOperatorStatistics{
        0,
        "Two-Opt Greedy",
        3,
        2,
        1,
        0,
        2,
        1,
        1,
        1,
        2,
        -55,
        21.75,
    });

    std::vector<GAMRunReport> reports = {
        {1, 1234, 17},
    };

    const bool success = save_gam_statistics(run_dir.string(), datasets::f10, 1, statistics, reports);
    assert(success);

    const std::filesystem::path statistics_dir = run_dir / "F_10_statistics";
    const std::filesystem::path trace_path = statistics_dir / "trace.csv";
    const std::filesystem::path operators_path = statistics_dir / "operators.csv";
    assert(std::filesystem::exists(trace_path));
    assert(std::filesystem::exists(operators_path));

    std::stringstream trace_buffer;
    trace_buffer << std::ifstream(trace_path).rdbuf();
    const std::string trace_content = trace_buffer.str();
    assert(trace_content.find("runtime_ms") != std::string::npos);
    assert(trace_content.find("7.25") != std::string::npos);

    std::stringstream operators_buffer;
    operators_buffer << std::ifstream(operators_path).rdbuf();
    const std::string operators_content = operators_buffer.str();
    assert(operators_content.find("avg_runtime_ms") != std::string::npos);
    assert(operators_content.find("Two-Opt Greedy") != std::string::npos);

    std::filesystem::remove_all(run_dir);

    std::cout << "test_save_gam_statistics_writes_operator_runtime_outputs passed\n";
}

int main() {
    try {
        test_instance_loading();
        test_pure_drone_feasibility_cache_matches_distance_limit();
        test_nearest_neighbour_validity();
        test_greedy_drone_cover_validity();
        test_one_reinsert_validity();
        test_one_reinsert_bounds();
        test_objective_improvement();
        test_solution_consistency();
        test_three_opt_improves_truck_route();
        test_two_opt_greedy_improves_truck_route();
        test_two_opt_greedy_never_accepts_worse_full_objective();
        test_two_opt_first_improvement_improves_truck_route();
        test_two_opt_arrival_screened_improves_truck_route();
        test_two_opt_random_swaps_combined_customer_slots();
        test_linkern_solver_solves_current_truck_subset();
        test_concorde_linkern_operator_improves_truck_route();
        test_sample_contiguous_slot_indices_returns_block();
        test_three_opt_random_permutates_combined_customer_slots();
        test_or_opt_segment_relocate_moves_block();
        test_or_opt_segment_relocate_first_improvement_improves_route();
        test_or_opt_segment_relocate_remaps_drone_anchor_indices();
        test_replace_drone_delivery_moves_customer_back_to_truck();
        test_replace_drone_delivery_greedy_moves_customer_back_to_truck();
        test_drone_demotion_shake_moves_customer_back_to_truck();
        test_replace_drone_delivery_picks_best_objective_insertion();
        test_nearest_neighbour_reassign_repairs_swapped_anchor_flights();
        test_replace_truck_delivery_repairs_removed_anchor_flights();
        test_alns_operator_applies_remove_then_insert();
        test_alns_operator_rolls_back_failed_insert();
        test_alns_pair_combination_materializes_named_operators();
        test_adaptive_composite_operator_rolls_back_failed_insert();
        test_gam_escape_algorithm_tracks_best_seen_while_returning_endpoint();
        test_gam_solution_cache_marks_first_visit_and_reuses_evaluation();
        test_gam_solution_cache_verify_with_master_check_rejects_infeasible_solution();
        test_shaw_removal_greedy_repair_random_medium_preserves_feasibility_on_success();
        test_exchange_k_large_preserves_feasibility_on_success();
        test_two_opt_repairs_drone_schedule_after_route_reversal();
        test_collect_two_opt_affected_drone_flights_returns_intersections();
        test_repair_after_two_opt_localized_repairs_candidate();
        test_solution_visualization_writes_jpg();
        test_drone_rendezvous_shift_moves_flight_within_window();
        test_drone_rendezvous_shift_first_improvement_applies_move();
        test_drone_rendezvous_shift_best_improvement_picks_best_flight();
        test_drone_rendezvous_shift_best_improvement_returns_best_feasible_move();
        test_single_drone_planner_shake_returns_false_without_drone_customers();
        test_single_drone_planner_shake_changes_one_drone_schedule();
        test_replace_drone_delivery_targeted_moves_customer_to_truck();
        test_exact_segment_reopt_uses_precomputed_pure_drone_cache();
        test_exact_segment_reopt_handles_suffix_window();
        test_exact_segment_reopt_random_small_retries_other_windows();
        test_save_gam_statistics_writes_operator_runtime_outputs();
        std::cout << "\nAll tests passed\n";
    } catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
