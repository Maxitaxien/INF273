#include "operators/substitute_truck_delivery.h"
#include "operators/helpers.h"
#include "operators/solution_fixers.h"
#include "verification/solution.h"
#include "datahandling/instance.h"
#include "general/sort_drone_collection.h"

#include <iostream>

bool substitute_truck_delivery(const Instance &inst, Solution &sol, int idx, int drone)
{
    if (idx < 0 || idx >= (int)sol.truck_route.size())
    {
        return false;
    }

    int customer = sol.truck_route[idx];

    // 1: POP
    pop_truck_delivery(sol, idx);

    // 2: INSERT - n = problem size / 10 index lookahead, pick best
    int look_ahead = inst.n / 10;

    // Sort the collection for this drone
    sort_drone_collection(sol.drones[drone]);

    auto [success, _] = assign_launch_and_land_n_lookahead(inst, sol, idx - 1, customer, drone, look_ahead);
    if (!success)
    {
        insert_truck_delivery(sol, customer, idx);
        return false;
    }

    // 3: Adjust indices of all drone launch-land pairs which happen after idx
    for (DroneCollection &drone_collection : sol.drones)
    {
        for (int i = 0; i < drone_collection.launch_indices.size(); i++)
        {
            if (drone_collection.launch_indices[i] > idx)
            {
                drone_collection.launch_indices[i]--;
            }

            if (drone_collection.land_indices[i] > idx)
            {
                drone_collection.land_indices[i]--;
            }
        }
    }
    return true;
}