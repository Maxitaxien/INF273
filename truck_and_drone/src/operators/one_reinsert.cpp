#include "operators/one_reinsert.h"
#include "operators/operator.h"
#include "algorithms/simple_initial_solution.h"
#include "verification/feasibility_check.h"
#include "operators/helpers.h"
#include "operators/solution_fixers.h"
#include <algorithm>
#include <iostream>
#include <random>

extern std::mt19937 gen;

bool one_reinsert(const Instance &inst, Solution &sol,
                  int pop, int insert, int idx)
{
    int value; // the node we're moving

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
            auto [ok, _] = greedy_assign_launch_and_land_assume_valid(inst, sol, value, d);
            inserted = ok;
        }
    }

    if (!inserted)
    { // restore original state
        if (pop == 1)
            sol.truck_route.push_back(value);
        else
            greedy_assign_launch_and_land_assume_valid(inst, sol, value, pop - 2);
        return false;
    }

    /* --- 3. keep the solution feasible/valid --- */
    sol = fix_overall_feasibility(inst, sol);
    return master_check(inst, sol, false);
}

std::vector<Solution> one_reinsert_operator(const Instance &instance, const Solution &sol)
{
    std::vector<Solution> neighbors;

    for (int pop = 1; pop <= 3; pop++)
    {
        for (int insert = 1; insert <= 3; insert++)
        {
            int truck_size_after_pop = sol.truck_route.size();
            if (pop == 1 && !sol.truck_route.empty())
            {
                truck_size_after_pop--;
            }

            for (int idx = 1; idx <= truck_size_after_pop; ++idx)
            {
                if (pop > 1 && (pop - 2 >= (int)(sol.drones.size()) || sol.drones[pop - 2].deliver_nodes.empty()))
                {
                    continue;
                }
                if (insert > 1 && (insert - 2 >= (int)(sol.drones.size())))
                {
                    continue;
                }

                Solution neighbor = sol;
                if (one_reinsert(instance, neighbor, pop, insert, idx))
                {
                    neighbors.push_back(neighbor);
                }
            }
        }
    }

    return neighbors;
}

bool one_reinsert_random(const Instance &instance, Solution &sol)
{
    std::uniform_int_distribution<int> pop_dist(1, 3);
    std::uniform_int_distribution<int> insert_dist(1, 3);

    int pop, insert;

    do
    {
        pop = pop_dist(gen);
        insert = insert_dist(gen);
    } while (
        (pop > 1 && (pop - 2 >= (int)(sol.drones.size()) || sol.drones[pop - 2].deliver_nodes.empty())) ||
        (insert > 1 && insert - 2 >= (int)(sol.drones.size())));

    int truck_size_after_pop = sol.truck_route.size();
    if (pop == 1 && !sol.truck_route.empty())
    {
        truck_size_after_pop--;
    }

    if (truck_size_after_pop <= 1)
    {
        return false;
    }

    std::uniform_int_distribution<int> idx_dist(1, truck_size_after_pop);
    const int idx = idx_dist(gen);

    return one_reinsert(instance, sol, pop, insert, idx);
}

bool one_reinsert_greedy(const Instance &instance, Solution &sol)
{
    (void)instance;
    (void)sol;
    return false;
}
