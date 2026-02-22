#include "operators/one_reinsert.h"
#include "algorithms/simple_initial_solution.h"
#include "verification/feasibility_check.h"
#include "operators/helpers.h"
#include <iostream>
#include <algorithm>

Solution one_reinsert(const Instance& instance, const Solution& solution, int pop, int insert, int idx) {
    Solution sol = solution;
    Solution original = sol;
    std::pair<bool, bool> drone_breaks_feasibility = {false, false};
    std::pair<bool, bool> drone_is_invalid = {false, false};

    // POP
    int to_insert;
    if (pop == 1) { // take from truck
        to_insert = sol.truck_route.back();
        sol.truck_route.pop_back();
        drone_is_invalid = drone_landed_at_back(sol);
    }
    else if (0 < pop  && pop < 4) { // take from drone
        int drone = pop - 2;
        to_insert = sol.drones[drone].deliver_nodes.back();

        sol.drones[drone].launch_indices.pop_back();
        sol.drones[drone].deliver_nodes.pop_back();
        sol.drones[drone].land_indices.pop_back();
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
        sol.truck_route.push_back(to_insert);
        if (idx > 0 && idx < sol.truck_route.size())
        {
            std::swap(sol.truck_route[sol.truck_route.size() - 1], sol.truck_route[idx]);
            // Any drones with a route overlapping the insertion index may be infeasible now, as they may
            // have to travel further. Fix the solution in this case.
            for (int c = 0; c < sol.drones.size(); c++) {
                std::set<Interval> drone_intervals = get_intervals(sol, c);
                int containing_flight_index = find_containing_interval_index(drone_intervals, idx);
                if (containing_flight_index != -1 && !specific_drone_flight_under_lim(instance, sol, c, containing_flight_index)) {
                    if (c == 0) drone_breaks_feasibility.first = true;
                    else if (c == 1) drone_breaks_feasibility.second = true;
                }
            }
        }
        else
        {
            std::cerr << "(1-Reinsert) Invalid idx for placement supplied: " << idx << "\n";
            std::cerr << "(1-Reinsert) Size of truck route is: " << sol.truck_route.size() << "\n";
            return original; 
        }
    }
    else if (0 < insert && insert < 4)
    {
        // If insert into drone: Put at end, then fix
        int drone = insert - 2;
        std::pair<bool, Solution> fix_result = assign_launch_and_land(instance, sol, to_insert, drone);
        if (!fix_result.first) {
            // std::cerr << "(1-Reinsert) insertion into drone failed: " << to_insert << "\n";
            return original;
        }
        sol = fix_result.second;
    }
    else
    {
        std::cerr << "(1-Reinsert) Invalid insert index supplied: " << insert << "\n";
        return original;
    }

    if (drone_breaks_feasibility.first && drone_breaks_feasibility.second) {
        sol = fix_overall_feasibility(instance, sol);
    }
    else if (drone_breaks_feasibility.first) {
       sol = fix_feasibility_for_drone(instance, sol, 0);
    }
    else if (drone_breaks_feasibility.second) {
        sol = fix_feasibility_for_drone(instance, sol, 1);
    }


    if (drone_is_invalid.first && drone_is_invalid.second) {
        sol = simple_fix_validity(sol);
    }
    else if (drone_is_invalid.first) {
        // solution = fix_validity(instance, solution, 0);
        sol = simple_fix_validity(sol);
    }
    else if (drone_is_invalid.second) {
        // solution = fix_validity(instance, solution, 1);
        sol = simple_fix_validity(sol);
    }

    return sol;
}