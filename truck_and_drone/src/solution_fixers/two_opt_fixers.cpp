#include "internal.h"

#include "general/sort_drone_collection.h"
#include "operators/helpers.h"
#include "verification/feasibility_check.h"
#include <algorithm>
#include <tuple>
#include <unordered_map>
#include <vector>

void remap_drone_anchor_indices_by_node(
    const Solution &before,
    Solution &after)
{
    std::unordered_map<int, int> new_truck_positions;
    new_truck_positions.reserve(after.truck_route.size());
    for (int idx = 0; idx < (int)(after.truck_route.size()); ++idx)
    {
        new_truck_positions[after.truck_route[idx]] = idx;
    }

    const int drone_count = std::min(
        (int)(before.drones.size()),
        (int)(after.drones.size()));
    for (int drone = 0; drone < drone_count; ++drone)
    {
        const DroneCollection &before_collection = before.drones[drone];
        DroneCollection &after_collection = after.drones[drone];
        const int flight_count = std::min({
            (int)(before_collection.launch_indices.size()),
            (int)(before_collection.land_indices.size()),
            (int)(after_collection.launch_indices.size()),
            (int)(after_collection.land_indices.size())});

        for (int flight_idx = 0; flight_idx < flight_count; ++flight_idx)
        {
            const int old_launch_idx = before_collection.launch_indices[flight_idx];
            const int old_land_idx = before_collection.land_indices[flight_idx];
            if (old_launch_idx < 0 || old_land_idx < 0 ||
                old_launch_idx >= (int)(before.truck_route.size()) ||
                old_land_idx >= (int)(before.truck_route.size()))
            {
                continue;
            }

            const int launch_node = before.truck_route[old_launch_idx];
            const int land_node = before.truck_route[old_land_idx];
            const auto new_launch_it = new_truck_positions.find(launch_node);
            const auto new_land_it = new_truck_positions.find(land_node);
            if (new_launch_it == new_truck_positions.end() ||
                new_land_it == new_truck_positions.end())
            {
                continue;
            }

            const int new_launch_idx = new_launch_it->second;
            const int new_land_idx = new_land_it->second;
            if (new_launch_idx >= new_land_idx)
            {
                continue;
            }

            after_collection.launch_indices[flight_idx] = new_launch_idx;
            after_collection.land_indices[flight_idx] = new_land_idx;
        }

        if (flight_count > 1)
        {
            sort_drone_collection(after_collection);
        }
    }
}

std::vector<AffectedDroneFlight> collect_two_opt_affected_drone_flights(
    const Solution &solution,
    int first,
    int second)
{
    std::vector<AffectedDroneFlight> affected;
    const int segment_start = first + 1;

    for (int drone = 0; drone < (int)(solution.drones.size()); ++drone)
    {
        const DroneCollection &dc = solution.drones[drone];
        const int flight_count = std::min({
            (int)(dc.launch_indices.size()),
            (int)(dc.land_indices.size()),
            (int)(dc.deliver_nodes.size())});

        for (int idx = 0; idx < flight_count; ++idx)
        {
            const int launch_idx = dc.launch_indices[idx];
            const int land_idx = dc.land_indices[idx];
            const int delivery = dc.deliver_nodes[idx];

            if (launch_idx <= second && land_idx >= segment_start)
            {
                affected.push_back({
                    drone,
                    idx,
                    delivery,
                    launch_idx,
                    land_idx,
                });
            }
        }
    }

    return affected;
}

bool repair_after_two_opt_localized(
    const Instance &instance,
    const Solution &before_move,
    int first,
    int second,
    Solution &candidate)
{
    Solution remapped_candidate = candidate;
    remap_drone_anchor_indices_by_node(before_move, remapped_candidate);
    if (master_check(instance, remapped_candidate, false))
    {
        candidate = std::move(remapped_candidate);
        return true;
    }

    std::vector<AffectedDroneFlight> affected =
        collect_two_opt_affected_drone_flights(before_move, first, second);
    if (affected.empty())
    {
        candidate = std::move(remapped_candidate);
        return false;
    }

    Solution working = remapped_candidate;

    std::sort(
        affected.begin(),
        affected.end(),
        [](const AffectedDroneFlight &lhs, const AffectedDroneFlight &rhs) {
            if (lhs.drone != rhs.drone)
            {
                return lhs.drone < rhs.drone;
            }

            return lhs.flight_idx > rhs.flight_idx;
        });

    std::vector<bool> touched_drones(working.drones.size(), false);
    for (const AffectedDroneFlight &flight : affected)
    {
        if (flight.drone < 0 || flight.drone >= (int)working.drones.size())
        {
            candidate = std::move(remapped_candidate);
            return false;
        }

        DroneCollection &drone_collection = working.drones[flight.drone];
        if (flight.flight_idx < 0 ||
            flight.flight_idx >= (int)drone_collection.deliver_nodes.size())
        {
            candidate = std::move(remapped_candidate);
            return false;
        }

        remove_drone_flight(working, flight.drone, flight.flight_idx);
        touched_drones[flight.drone] = true;
    }

    std::sort(
        affected.begin(),
        affected.end(),
        [](const AffectedDroneFlight &lhs, const AffectedDroneFlight &rhs) {
            return std::tie(lhs.launch_idx, lhs.land_idx, lhs.drone, lhs.flight_idx) <
                std::tie(rhs.launch_idx, rhs.land_idx, rhs.drone, rhs.flight_idx);
        });

    for (const AffectedDroneFlight &flight : affected)
    {
        if (delivery_on_truck_route(working, flight.delivery))
        {
            candidate = std::move(remapped_candidate);
            return false;
        }

        const RouteTiming timing = compute_route_timing(instance, working);
        const std::set<Interval> drone_intervals = get_intervals(working, flight.drone);
        const FlightAssignment assignment = find_two_opt_local_feasible_flight(
            instance,
            working,
            timing,
            drone_intervals,
            flight,
            first,
            second);
        if (!assignment.feasible)
        {
            candidate = std::move(remapped_candidate);
            return false;
        }

        append_drone_flight(working, flight.drone, flight.delivery, assignment);
        touched_drones[flight.drone] = true;
    }

    for (int drone = 0; drone < (int)touched_drones.size(); ++drone)
    {
        if (touched_drones[drone] &&
            (int)(working.drones[drone].deliver_nodes.size()) > 1)
        {
            sort_drone_collection(working.drones[drone]);
        }
    }

    if (!master_check(instance, working, false))
    {
        candidate = std::move(remapped_candidate);
        return false;
    }

    candidate = std::move(working);
    return true;
}
