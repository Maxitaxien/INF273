#include "operators/replace_drone_delivery.h"
#include "operators/helpers.h"
#include "verification/feasibility_check.h"
#include <algorithm>
#include <utility>
#include <vector>

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

long long estimate_truck_insert_delta(
    const Instance &inst,
    const Solution &sol,
    int delivery,
    int insert_idx)
{
    const int prev = sol.truck_route[insert_idx - 1];
    if (insert_idx == (int)(sol.truck_route.size()))
    {
        return inst.truck_matrix[prev][delivery];
    }

    const int next = sol.truck_route[insert_idx];
    return inst.truck_matrix[prev][delivery] +
        inst.truck_matrix[delivery][next] -
        inst.truck_matrix[prev][next];
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

    std::vector<std::pair<long long, int>> candidate_positions;
    candidate_positions.reserve(insert_end - insert_start + 1);
    for (int insert_idx = insert_start; insert_idx <= insert_end; ++insert_idx)
    {
        candidate_positions.emplace_back(
            estimate_truck_insert_delta(inst, sol, delivery, insert_idx),
            insert_idx);
    }

    std::sort(candidate_positions.begin(), candidate_positions.end());

    for (const auto &[heuristic_delta, insert_idx] : candidate_positions)
    {
        (void)heuristic_delta;

        Solution candidate = sol;
        remove_drone_flight(candidate, drone, flight_idx);
        candidate.truck_route.insert(candidate.truck_route.begin() + insert_idx, delivery);
        shift_drone_indices_after_truck_insert(candidate, insert_idx);

        if (master_check(inst, candidate, false))
        {
            sol = std::move(candidate);
            return true;
        }
    }

    return false;
}
