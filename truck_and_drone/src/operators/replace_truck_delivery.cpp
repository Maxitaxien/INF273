#include "operators/replace_truck_delivery.h"
#include "datahandling/instance.h"
#include "general/sort_drone_collection.h"
#include "operators/helpers.h"
#include "operators/solution_fixers.h"
#include "verification/feasibility_check.h"
#include "verification/solution.h"

bool replace_truck_delivery(const Instance &inst, Solution &sol, int idx, int drone)
{
    if (idx <= 0 || idx >= (int)(sol.truck_route.size()))
    {
        return false;
    }
    if (drone < 0 || drone >= (int)(sol.drones.size()))
    {
        return false;
    }

    int customer = sol.truck_route[idx];

    // 1: POP
    pop_truck_delivery(sol, idx);

    // 2: Adjust indices of all drone launch-land pairs which happen after idx.
    // Must be done before assigning new flight so we don't shift the new indices.
    for (DroneCollection &drone_collection : sol.drones)
    {
        const int flight_count = (int)(drone_collection.launch_indices.size());
        for (int i = 0; i < flight_count; ++i)
        {
            if (drone_collection.launch_indices[i] >= idx)
            {
                drone_collection.launch_indices[i]--;
            }

            if (drone_collection.land_indices[i] >= idx)
            {
                drone_collection.land_indices[i]--;
            }
        }
    }

    // 3: INSERT - n = problem size / 10 index lookahead, pick best
    int look_ahead = inst.n / 10;
    sort_drone_collection(sol.drones[drone]);

    auto [success, ignored_solution] = assign_launch_and_land_n_lookahead(inst, sol, idx - 1, customer, drone, look_ahead);
    (void)ignored_solution;
    if (!success)
    {
        insert_truck_delivery(sol, customer, idx);
        return false;
    }

    sol = fix_overall_feasibility(inst, sol);
    return master_check(inst, sol, false);
}

