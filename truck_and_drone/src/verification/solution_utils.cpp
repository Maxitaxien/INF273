#include "verification/solution.h"

#include "datahandling/instance.h"
#include "datahandling/instance_preprocessing.h"

#include <algorithm>
#include <tuple>

namespace
{
int flight_count(const DroneCollection &collection)
{
    return std::min({
        (int)collection.launch_indices.size(),
        (int)collection.deliver_nodes.size(),
        (int)collection.land_indices.size()});
}

int latest_flight_index(const DroneCollection &collection)
{
    const int count = flight_count(collection);
    int best_idx = -1;

    for (int idx = 0; idx < count; ++idx)
    {
        if (best_idx < 0 ||
            std::tie(
                collection.launch_indices[idx],
                collection.land_indices[idx]) >
                std::tie(
                    collection.launch_indices[best_idx],
                    collection.land_indices[best_idx]))
        {
            best_idx = idx;
        }
    }

    return best_idx;
}
} // namespace

bool same_solution(const Solution &lhs, const Solution &rhs)
{
    if (lhs.truck_route != rhs.truck_route ||
        lhs.drones.size() != rhs.drones.size())
    {
        return false;
    }

    for (int drone = 0; drone < (int)lhs.drones.size(); ++drone)
    {
        const DroneCollection &lhs_collection = lhs.drones[drone];
        const DroneCollection &rhs_collection = rhs.drones[drone];
        if (lhs_collection.launch_indices != rhs_collection.launch_indices ||
            lhs_collection.deliver_nodes != rhs_collection.deliver_nodes ||
            lhs_collection.land_indices != rhs_collection.land_indices)
        {
            return false;
        }
    }

    return true;
}

int terminal_depot_land_index(const Solution &solution)
{
    return (int)solution.truck_route.size();
}

bool is_terminal_depot_landing(const Solution &solution, int land_idx)
{
    return land_idx == terminal_depot_land_index(solution);
}

Solution canonicalize_terminal_depot_landings(
    const Instance &instance,
    Solution solution)
{
    const int route_size = (int)solution.truck_route.size();
    if (route_size <= 1)
    {
        return solution;
    }

    for (DroneCollection &collection : solution.drones)
    {
        const int last_idx = latest_flight_index(collection);
        if (last_idx < 0)
        {
            continue;
        }

        const int launch_idx = collection.launch_indices[last_idx];
        const int delivery = collection.deliver_nodes[last_idx];
        if (launch_idx < 0 || launch_idx >= route_size ||
            delivery <= 0 || delivery > instance.n)
        {
            continue;
        }

        const int launch_node = solution.truck_route[launch_idx];
        if (pure_drone_flight_within_limit(instance, launch_node, delivery, 0))
        {
            collection.land_indices[last_idx] = route_size;
        }
    }

    return solution;
}
