#include "internal.h"

#include "general/sort_drone_collection.h"
#include "operators/helpers.h"
#include <algorithm>
#include <unordered_set>
#include <utility>

Solution &fix_feasibility_for_drone(const Instance &instance, Solution &sol, int drone)
{
    if (drone < 0 || drone >= (int)sol.drones.size())
    {
        return sol;
    }

    for (int i = 0; i < (int)(sol.drones[drone].launch_indices.size());)
    {
        const RouteTiming timing = compute_route_timing(instance, sol);
        std::set<Interval> intervals = get_intervals(sol, drone);

        const int launch_idx = sol.drones[drone].launch_indices[i];
        const int land_idx = sol.drones[drone].land_indices[i];
        const int deliver = sol.drones[drone].deliver_nodes[i];

        bool bad = !flight_under_limit_with_wait(instance, sol, timing, drone, i);
        intervals.erase(Interval{launch_idx, land_idx});
        if (launch_idx >= land_idx || overlaps(intervals, launch_idx, land_idx))
        {
            bad = true;
        }

        if (!bad)
        {
            ++i;
            continue;
        }

        remove_drone_flight(sol, drone, i);
        auto [success, ignored_solution] = greedy_assign_launch_and_land_assume_valid(
            instance,
            sol,
            deliver,
            drone);
        (void)ignored_solution;
        if (!success)
        {
            const int truck_pos = std::min(
                launch_idx + 1,
                (int)sol.truck_route.size());
            sol.truck_route.insert(sol.truck_route.begin() + truck_pos, deliver);
        }
    }

    if ((int)(sol.drones[drone].deliver_nodes.size()) > 1)
    {
        sort_drone_collection(sol.drones[drone]);
    }

    return sol;
}

Solution &simple_fix_validity(Solution &solution)
{
    const int route_size = (int)(solution.truck_route.size());
    const int final_index = route_size - 1;
    std::unordered_set<int> truck_customers;
    std::unordered_set<int> kept_drone_customers;
    std::unordered_set<int> queued_for_truck;
    std::vector<int> to_reinsert;

    for (int node : solution.truck_route)
    {
        if (node != 0)
        {
            truck_customers.insert(node);
        }
    }

    for (DroneCollection &drone_collection : solution.drones)
    {
        int last_flight_idx = -1;
        for (int idx = 0; idx < (int)(drone_collection.deliver_nodes.size()); ++idx)
        {
            if (last_flight_idx < 0 ||
                std::pair{
                    drone_collection.launch_indices[idx],
                    drone_collection.land_indices[idx]} >
                    std::pair{
                        drone_collection.launch_indices[last_flight_idx],
                        drone_collection.land_indices[last_flight_idx]})
            {
                last_flight_idx = idx;
            }
        }

        for (int i = 0; i < (int)(drone_collection.deliver_nodes.size());)
        {
            const int launch = drone_collection.launch_indices[i];
            const int landing = drone_collection.land_indices[i];
            const int delivery = drone_collection.deliver_nodes[i];
            const bool terminal_depot =
                landing == terminal_depot_land_index(solution);
            const bool invalid_index =
                route_size < 2 ||
                launch < 0 || landing < 0 ||
                launch >= route_size || landing > route_size ||
                launch >= landing ||
                (!terminal_depot && landing >= final_index) ||
                (terminal_depot && i != last_flight_idx);
            const bool duplicate_on_truck =
                delivery <= 0 || truck_customers.find(delivery) != truck_customers.end();
            const bool duplicate_on_drone =
                !duplicate_on_truck &&
                !kept_drone_customers.insert(delivery).second;

            if (!invalid_index && !duplicate_on_truck && !duplicate_on_drone)
            {
                ++i;
                continue;
            }

            if (!duplicate_on_truck && !duplicate_on_drone &&
                queued_for_truck.insert(delivery).second)
            {
                to_reinsert.push_back(delivery);
            }

            drone_collection.launch_indices.erase(
                drone_collection.launch_indices.begin() + i);
            drone_collection.land_indices.erase(
                drone_collection.land_indices.begin() + i);
            drone_collection.deliver_nodes.erase(
                drone_collection.deliver_nodes.begin() + i);
        }
    }

    for (int delivery : to_reinsert)
    {
        if (kept_drone_customers.find(delivery) == kept_drone_customers.end() &&
            truck_customers.insert(delivery).second)
        {
            solution.truck_route.push_back(delivery);
        }
    }

    return solution;
}
