#include "operators/one_reinsert.h"
#include "algorithms/simple_initial_solution.h"
#include "verification/feasibility_check.h"
#include "operators/helpers.h"
#include "operators/solution_fixers.h"
#include <iostream>
#include <algorithm>

bool one_reinsert(const Instance &inst, Solution &sol,
                  int pop, int insert, int idx)
{
    int value; // the node we’re moving

    /* --- 1. remove the element --- */
    if (pop == 1)
    { // truck tail
        if (sol.truck_route.empty())
            return false;
        value = pop_truck_delivery(sol, sol.truck_route.size() - 1);
    }
    else if (pop >= 2 && pop <= 3)
    { // drone 0 or 1
        int d = pop - 2;
        if (d >= (int)sol.drones.size() ||
            sol.drones[d].deliver_nodes.empty())
            return false;
        value = sol.drones[d].deliver_nodes.back();
        remove_drone_flight(sol, d); // helper erases all three vectors
    }
    else
    {
        return false;
    }

    /* --- 2. insert the value --- */
    bool inserted = false;
    if (insert == 1)
    { // truck insertion
        // move last element to idx
        if (idx <= 0 || idx >= (int)sol.truck_route.size())
        {
            inserted = false;
        }
        else
        {
            sol.truck_route.push_back(value);
            std::swap(sol.truck_route[idx], sol.truck_route.back());
            inserted = true;
        }
    }
    else if (insert >= 2 && insert <= 3)
    { // drone insertion
        int d = insert - 2;
        if (d < (int)sol.drones.size())
        {
            auto [ok, _] = greedy_assign_launch_and_land(inst, sol, value, d);
            inserted = ok;
        }
    }

    if (!inserted)
    { // restore original state
        if (pop == 1)
            sol.truck_route.push_back(value);
        else
            greedy_assign_launch_and_land(inst, sol, value, pop - 2);
        return false;
    }

    /* --- 3. keep the solution feasible/valid --- */
    fix_overall_feasibility(inst, sol); // cheap enough to always run
    simple_fix_validity(sol);
    return true;
}