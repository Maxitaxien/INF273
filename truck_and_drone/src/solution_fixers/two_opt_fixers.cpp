#include "internal.h"

#include "general/sort_drone_collection.h"
#include "operators/helpers.h"
#include "verification/feasibility_check.h"
#include <algorithm>
#include <functional>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace
{
std::vector<AffectedDroneFlight> collect_affected_drone_flights_if(
    const Solution &solution,
    const std::function<bool(int, int)> &predicate)
{
    std::vector<AffectedDroneFlight> affected;

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
            if (!predicate(launch_idx, land_idx))
            {
                continue;
            }

            affected.push_back({
                drone,
                idx,
                dc.deliver_nodes[idx],
                launch_idx,
                land_idx,
            });
        }
    }

    return affected;
}

bool repair_affected_drone_flights_impl(
    const Instance &instance,
    const Solution &before_move,
    const std::vector<AffectedDroneFlight> &affected,
    Solution &candidate,
    const std::vector<int> &allowed_missing,
    const std::function<FlightAssignment(
        const Solution &,
        const AffectedDroneFlight &)> &select_assignment)
{
    const auto candidate_passes_check =
        [&instance, &allowed_missing](const Solution &solution) {
            if (allowed_missing.empty())
            {
                return master_check(instance, solution, false);
            }

            Solution completed = solution;
            for (int delivery : allowed_missing)
            {
                if (delivery <= 0)
                {
                    continue;
                }

                if (delivery_on_truck_route(completed, delivery))
                {
                    continue;
                }

                bool delivered_by_drone = false;
                for (const DroneCollection &drone : completed.drones)
                {
                    if (std::find(
                            drone.deliver_nodes.begin(),
                            drone.deliver_nodes.end(),
                            delivery) != drone.deliver_nodes.end())
                    {
                        delivered_by_drone = true;
                        break;
                    }
                }

                if (!delivered_by_drone)
                {
                    completed.truck_route.push_back(delivery);
                }
            }

            return master_check(instance, completed, false);
        };

    Solution remapped_candidate = candidate;
    remap_drone_anchor_indices_by_node(before_move, remapped_candidate);
    if (candidate_passes_check(remapped_candidate))
    {
        candidate = std::move(remapped_candidate);
        return true;
    }

    if (affected.empty())
    {
        candidate = std::move(remapped_candidate);
        return false;
    }

    Solution working = remapped_candidate;
    std::vector<AffectedDroneFlight> affected_desc = affected;
    std::sort(
        affected_desc.begin(),
        affected_desc.end(),
        [](const AffectedDroneFlight &lhs, const AffectedDroneFlight &rhs) {
            if (lhs.drone != rhs.drone)
            {
                return lhs.drone < rhs.drone;
            }

            return lhs.flight_idx > rhs.flight_idx;
        });

    std::vector<bool> touched_drones(working.drones.size(), false);
    for (const AffectedDroneFlight &flight : affected_desc)
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

    std::vector<AffectedDroneFlight> affected_asc = affected;
    std::sort(
        affected_asc.begin(),
        affected_asc.end(),
        [](const AffectedDroneFlight &lhs, const AffectedDroneFlight &rhs) {
            return std::tie(lhs.launch_idx, lhs.land_idx, lhs.drone, lhs.flight_idx) <
                std::tie(rhs.launch_idx, rhs.land_idx, rhs.drone, rhs.flight_idx);
        });

    for (const AffectedDroneFlight &flight : affected_asc)
    {
        if (delivery_on_truck_route(working, flight.delivery))
        {
            candidate = std::move(remapped_candidate);
            return false;
        }

        const FlightAssignment assignment = select_assignment(working, flight);
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

    if (!candidate_passes_check(working))
    {
        candidate = std::move(remapped_candidate);
        return false;
    }

    candidate = std::move(working);
    return true;
}
} // namespace

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
    const int segment_start = first + 1;
    return collect_affected_drone_flights_if(
        solution,
        [segment_start, second](int launch_idx, int land_idx) {
            return launch_idx <= second && land_idx >= segment_start;
        });
}

std::vector<AffectedDroneFlight> collect_swap_affected_drone_flights(
    const Solution &solution,
    int first,
    int second)
{
    const int left = std::min(first, second);
    const int right = std::max(first, second);
    return collect_affected_drone_flights_if(
        solution,
        [left, right](int launch_idx, int land_idx) {
            return launch_idx <= right && land_idx >= left;
        });
}

std::vector<AffectedDroneFlight> collect_removed_anchor_drone_flights(
    const Solution &solution,
    int removed_idx)
{
    return collect_affected_drone_flights_if(
        solution,
        [removed_idx](int launch_idx, int land_idx) {
            return launch_idx == removed_idx || land_idx == removed_idx;
        });
}

bool repair_affected_drone_flights_localized(
    const Instance &instance,
    const Solution &before_move,
    const std::vector<AffectedDroneFlight> &affected,
    Solution &candidate,
    const std::vector<int> &allowed_missing)
{
    return repair_affected_drone_flights_impl(
        instance,
        before_move,
        affected,
        candidate,
        allowed_missing,
        [&instance](const Solution &working, const AffectedDroneFlight &flight) {
            const RouteTiming timing = compute_route_timing(instance, working);
            const std::set<Interval> drone_intervals =
                get_intervals(working, flight.drone);
            return find_vicinity_feasible_flight(
                instance,
                working,
                timing,
                drone_intervals,
                flight.delivery,
                flight.drone);
        });
}

bool repair_after_two_opt_localized(
    const Instance &instance,
    const Solution &before_move,
    int first,
    int second,
    Solution &candidate)
{
    std::vector<AffectedDroneFlight> affected =
        collect_two_opt_affected_drone_flights(before_move, first, second);
    return repair_affected_drone_flights_impl(
        instance,
        before_move,
        affected,
        candidate,
        {},
        [&instance, first, second](
            const Solution &working,
            const AffectedDroneFlight &flight) {
            const RouteTiming timing = compute_route_timing(instance, working);
            const std::set<Interval> drone_intervals =
                get_intervals(working, flight.drone);
            return find_two_opt_local_feasible_flight(
                instance,
                working,
                timing,
                drone_intervals,
                flight,
                first,
                second);
        });
}
