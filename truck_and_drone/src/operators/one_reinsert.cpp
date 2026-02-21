#include "operators/one_reinsert.h"
#include "algorithms/simple_initial_solution.h"
#include "verification/feasibility_check.h"
#include "operators/helpers.h"
#include <iostream>
#include <algorithm>

Solution one_reinsert(const Instance& instance, Solution& solution, int pop, int insert, int idx) {
    Solution original = solution;
    std::pair<bool, bool> drone_breaks_feasibility = {false, false};
    std::pair<bool, bool> drone_is_invalid = {false, false};

    // POP
    int to_insert;
    if (pop == 1) { // take from truck
        to_insert = solution.truck_route.back();
        solution.truck_route.pop_back();
        drone_is_invalid = drone_landed_at_back(solution);
    }
    else if (0 < pop  && pop < 4) { // take from drone
        int drone = pop - 2;
        to_insert = solution.drones[drone].deliver_nodes.back();

        solution.drones[drone].launch_indices.pop_back();
        solution.drones[drone].deliver_nodes.pop_back();
        solution.drones[drone].land_indices.pop_back();
    }
    else
    {
        std::cerr << "(1-Reinsert) Invalid pop index supplied: " << pop << "\n";
        return original;
    }

    // REINSERT
    if (insert == 1)
    {
        // If insert into truck: Put at end, then swap element at idx.
        solution.truck_route.push_back(to_insert);
        if (idx > 0 && idx < solution.truck_route.size())
        {
            std::swap(solution.truck_route[solution.truck_route.size() - 1], solution.truck_route[idx]);
            // Any drones with a route overlapping the insertion index may be infeasible now, as they may
            // have to travel further. Fix the solution in this case.
            for (int c = 0; c < solution.drones.size(); c++) {
                std::set<Interval> drone_intervals = get_intervals(solution, c);
                int containing_flight_index = find_containing_interval_index(drone_intervals, idx);
                if (containing_flight_index != -1 && !specific_drone_flight_under_lim(instance, solution, c, containing_flight_index)) {
                    if (c == 0) drone_breaks_feasibility.first = true;
                    else if (c == 1) drone_breaks_feasibility.second = true;
                }
            }
        }
        else
        {
            std::cerr << "(1-Reinsert) Invalid idx for placement supplied: " << idx << "\n";
            std::cerr << "(1-Reinsert) Size of truck route is: " << solution.truck_route.size() << "\n";
            return original; 
        }
    }
    else if (0 < insert && insert < 4)
    {
        // If insert into drone: Put at end, then fix
        int drone = insert - 2;
        std::pair<bool, Solution> fix_result = assign_launch_and_land(instance, solution, to_insert, drone);
        if (!fix_result.first) {
            std::cerr << "(1-Reinsert) insertion into drone failed: " << to_insert << "\n";
            return original;
        }
        solution = fix_result.second;
    }
    else
    {
        std::cerr << "(1-Reinsert) Invalid insert index supplied: " << insert << "\n";
        return original;
    }

    if (drone_breaks_feasibility.first && drone_breaks_feasibility.second) {
        solution = fix_overall_feasibility(instance, solution);
    }
    else if (drone_breaks_feasibility.first) {
       solution = fix_feasibility_for_drone(instance, solution, 0);
    }
    else if (drone_breaks_feasibility.second) {
        solution = fix_feasibility_for_drone(instance, solution, 1);
    }


    if (drone_is_invalid.first && drone_is_invalid.second) {
        solution = simple_fix_validity(solution);
    }
    else if (drone_is_invalid.first) {
        // solution = fix_validity(instance, solution, 0);
        solution = simple_fix_validity(solution);
    }
    else if (drone_is_invalid.second) {
        // solution = fix_validity(instance, solution, 1);
        solution = simple_fix_validity(solution);
    }

    return solution;
}