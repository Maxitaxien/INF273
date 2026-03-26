#include "operators/replace_drone_delivery.h"
#include "operators/helpers.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <limits>

namespace
{
void shift_drone_indices_after_truck_insert(Solution &sol, int insert_idx)
{
    for (DroneCollection &drone_collection : sol.drones)
    {
        const int flight_count = (int)(drone_collection.launch_indices.size());
        for (int i = 0; i < flight_count; ++i)
        {
            if (drone_collection.launch_indices[i] >= insert_idx)
            {
                ++drone_collection.launch_indices[i];
            }

            if (drone_collection.land_indices[i] >= insert_idx)
            {
                ++drone_collection.land_indices[i];
            }
        }
    }
}
}

bool replace_drone_delivery(
    const Instance &inst,
    Solution &sol,
    int drone,
    int flight_idx)
{
    if (drone < 0 || drone >= (int)(sol.drones.size()))
    {
        return false;
    }

    const DroneCollection &drone_collection = sol.drones[drone];
    if (flight_idx < 0 || flight_idx >= (int)(drone_collection.deliver_nodes.size()))
    {
        return false;
    }

    const int route_size = (int)(sol.truck_route.size());
    const int launch_idx = drone_collection.launch_indices[flight_idx];
    const int land_idx = drone_collection.land_indices[flight_idx];
    const int delivery = drone_collection.deliver_nodes[flight_idx];
    if (route_size < 2 || launch_idx < 0 || land_idx < 0 || launch_idx >= land_idx)
    {
        return false;
    }

    const int insert_start = std::max(1, launch_idx + 1);
    const int insert_end = std::min(route_size, land_idx + 1);
    if (insert_start > insert_end)
    {
        return false;
    }

    bool found_candidate = false;
    long long best_objective = std::numeric_limits<long long>::max();
    Solution best_solution;
    Solution candidate = sol;

    for (int insert_idx = insert_start; insert_idx <= insert_end; ++insert_idx)
    {
        candidate = sol;
        remove_drone_flight(candidate, drone, flight_idx);
        candidate.truck_route.insert(candidate.truck_route.begin() + insert_idx, delivery);
        shift_drone_indices_after_truck_insert(candidate, insert_idx);

        if (!master_check(inst, candidate, false))
        {
            continue;
        }

        const long long objective = objective_function_impl(inst, candidate);
        if (!found_candidate || objective < best_objective)
        {
            found_candidate = true;
            best_objective = objective;
            best_solution = std::move(candidate);
        }
    }

    if (!found_candidate)
    {
        return false;
    }

    sol = std::move(best_solution);
    return true;
}
