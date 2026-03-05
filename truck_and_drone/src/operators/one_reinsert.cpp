#include "operators/one_reinsert.h"
#include "algorithms/simple_initial_solution.h"
#include "verification/feasibility_check.h"
#include "operators/helpers.h"
#include "operators/solution_fixers.h"
#include <iostream>
#include <algorithm>

bool one_reinsert(const Instance &instance, Solution &sol, int pop, int insert, int idx)
{
    std::pair<bool, bool> drone_breaks_feasibility = {false, false};
    std::pair<bool, bool> drone_is_invalid = {false, false};

    int to_insert;

    // POP
    if (pop == 1)
    { // truck
        if (sol.truck_route.empty())
            return false;
        to_insert = sol.truck_route.back();
        sol.truck_route.pop_back();
        drone_is_invalid = drone_landed_at_back(sol);
    }
    else if (pop > 1 && pop < 4)
    { // drone
        int drone = pop - 2;
        DroneCollection &dc = sol.drones[drone];
        if (dc.deliver_nodes.empty())
            return false;
        to_insert = dc.deliver_nodes.back();
        dc.launch_indices.pop_back();
        dc.deliver_nodes.pop_back();
        dc.land_indices.pop_back();
    }
    else
        return false;

    // REINSERT
    if (insert == 1)
    { // truck
        sol.truck_route.push_back(to_insert);
        if (idx > 0 && idx < sol.truck_route.size())
        {
            std::swap(sol.truck_route.back(), sol.truck_route[idx]);
        }
        else
            return false;
    }
    else if (insert > 1 && insert < 4)
    { // drone
        int drone = insert - 2;
        auto [success, _] = greedy_assign_launch_and_land(instance, sol, to_insert, drone);
        if (!success)
            return false;
    }
    else
        return false;

    // fix feasibility & validity in-place
    if (drone_breaks_feasibility.first && drone_breaks_feasibility.second)
        fix_overall_feasibility(instance, sol);
    else if (drone_breaks_feasibility.first)
        fix_feasibility_for_drone(instance, sol, 0);
    else if (drone_breaks_feasibility.second)
        fix_feasibility_for_drone(instance, sol, 1);

    if (drone_is_invalid.first || drone_is_invalid.second)
        simple_fix_validity(sol);

    return true;
}