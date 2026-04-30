#include "internal.h"

#include "datahandling/instance_preprocessing.h"
#include "general/get_customer_positions.h"
#include "operators/helpers.h"
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <vector>

bool delivery_on_truck_route(const Solution &solution, int delivery)
{
    return std::find(
               solution.truck_route.begin(),
               solution.truck_route.end(),
               delivery) != solution.truck_route.end();
}

AssignmentSearchLimits make_assignment_search_limits(
    const Instance &instance,
    int route_size,
    bool expanded)
{
    const int feasible_positions = std::max(1, route_size - 1);
    if (!expanded)
    {
        return AssignmentSearchLimits{
            std::min(6, feasible_positions),
            std::min(2, feasible_positions),
            std::min(6, std::max(3, instance.n / 20)),
        };
    }

    return AssignmentSearchLimits{
        std::min(10, feasible_positions),
        std::min(4, feasible_positions),
        std::min(10, std::max(6, instance.n / 10)),
    };
}

bool flight_under_limit_with_wait(
    const Instance &instance,
    const Solution &solution,
    const RouteTiming &timing,
    int drone,
    int flight_idx)
{
    const DroneCollection &collection = solution.drones[drone];
    const int route_size = (int)(solution.truck_route.size());
    const int launch_idx = collection.launch_indices[flight_idx];
    const int land_idx = collection.land_indices[flight_idx];
    const int deliver = collection.deliver_nodes[flight_idx];

    if (launch_idx < 0 || land_idx < 0 || deliver <= 0 ||
        launch_idx >= route_size || land_idx > route_size ||
        launch_idx >= land_idx)
    {
        return false;
    }

    const int launch_node = solution.truck_route[launch_idx];
    const int land_node =
        is_terminal_depot_landing(solution, land_idx)
            ? 0
            : solution.truck_route[land_idx];
    if (!pure_drone_flight_within_limit(instance, launch_node, deliver, land_node))
    {
        return false;
    }

    const long long launch_time = std::max(
        timing.truck_arrival[launch_idx],
        timing.drone_ready_at_stop[drone][launch_idx]);
    const long long out_time = instance.drone_matrix[launch_node][deliver];
    const long long back_time = instance.drone_matrix[deliver][land_node];
    const long long drone_return = launch_time + out_time + back_time;
    const long long drone_wait =
        is_terminal_depot_landing(solution, land_idx) || land_node == 0
        ? 0
        : std::max(timing.truck_arrival[land_idx] - drone_return, 0LL);

    return out_time + back_time + drone_wait <= instance.lim;
}

void consider_flight_assignment(
    const Instance &instance,
    const Solution &solution,
    const RouteTiming &timing,
    const std::set<Interval> &drone_intervals,
    int delivery,
    int drone,
    int launch_idx,
    int land_idx,
    FlightAssignment &best)
{
    const int route_size = (int)(solution.truck_route.size());
    if (route_size < 3 ||
        launch_idx < 0 || land_idx < 0 ||
        launch_idx >= route_size || land_idx >= route_size ||
        launch_idx >= land_idx ||
        overlaps(drone_intervals, launch_idx, land_idx))
    {
        return;
    }

    const int launch_node = solution.truck_route[launch_idx];
    const int land_node = solution.truck_route[land_idx];
    if (!pure_drone_flight_within_limit(instance, launch_node, delivery, land_node))
    {
        return;
    }

    const long long launch_time = std::max(
        timing.truck_arrival[launch_idx],
        timing.drone_ready_at_stop[drone][launch_idx]);
    const long long out_time = instance.drone_matrix[launch_node][delivery];
    const long long back_time = instance.drone_matrix[delivery][land_node];
    const long long drone_arrival = launch_time + out_time;
    const long long drone_return = drone_arrival + back_time;
    const long long drone_wait = std::max(
        timing.truck_arrival[land_idx] - drone_return,
        0LL);
    const long long total_with_wait = out_time + back_time + drone_wait;
    if (total_with_wait > instance.lim)
    {
        return;
    }

    const long long truck_wait = std::max(
        drone_return - timing.truck_arrival[land_idx],
        0LL);
    const long long downstream_stops = std::max(0, route_size - land_idx - 1);
    const long long downstream_delay_impact = truck_wait * downstream_stops;

    const auto candidate_key = std::tie(
        downstream_delay_impact,
        truck_wait,
        drone_wait,
        drone_arrival,
        land_idx);

    const auto best_key = std::tie(
        best.downstream_delay_impact,
        best.truck_wait,
        best.drone_wait,
        best.drone_arrival,
        best.land_idx);

    if (!best.feasible || candidate_key < best_key)
    {
        best.feasible = true;
        best.launch_idx = launch_idx;
        best.land_idx = land_idx;
        best.downstream_delay_impact = downstream_delay_impact;
        best.truck_wait = truck_wait;
        best.drone_wait = drone_wait;
        best.drone_arrival = drone_arrival;
    }
}

FlightAssignment find_vicinity_feasible_flight(
    const Instance &instance,
    const Solution &solution,
    const RouteTiming &timing,
    const std::set<Interval> &drone_intervals,
    int delivery,
    int drone)
{
    FlightAssignment best;
    const int route_size = (int)(solution.truck_route.size());
    if (route_size < 3)
    {
        return best;
    }

    const std::unordered_map<int, int> customer_positions = get_customer_positions(solution);
    std::vector<int> anchor_positions;
    std::unordered_set<int> seen_positions;
    anchor_positions.reserve(solution.truck_route.size());

    anchor_positions.push_back(0);
    seen_positions.insert(0);

    for (int node : sort_by_distance_to_point_drone(instance, solution, delivery))
    {
        auto pos_it = customer_positions.find(node);
        if (pos_it == customer_positions.end())
        {
            continue;
        }

        const int anchor_idx = pos_it->second;
        if (anchor_idx >= route_size - 1 || !seen_positions.insert(anchor_idx).second)
        {
            continue;
        }

        anchor_positions.push_back(anchor_idx);
    }

    for (bool expanded : {false, true})
    {
        const AssignmentSearchLimits limits =
            make_assignment_search_limits(instance, route_size, expanded);
        const int anchors_to_try = std::min(
            (int)(anchor_positions.size()),
            limits.anchor_count);

        for (int anchor_i = 0; anchor_i < anchors_to_try; ++anchor_i)
        {
            const int anchor_idx = anchor_positions[anchor_i];
            const int launch_start = std::max(0, anchor_idx - limits.launch_back_window);
            const int launch_end = std::min(anchor_idx, route_size - 2);
            const int land_end = std::min(route_size - 2, anchor_idx + limits.land_forward_window);

            for (int launch_idx = launch_start; launch_idx <= launch_end; ++launch_idx)
            {
                const int land_start = std::max(launch_idx + 1, anchor_idx);
                for (int land_idx = land_start; land_idx <= land_end; ++land_idx)
                {
                    consider_flight_assignment(
                        instance,
                        solution,
                        timing,
                        drone_intervals,
                        delivery,
                        drone,
                        launch_idx,
                        land_idx,
                        best);
                }
            }
        }

        if (best.feasible)
        {
            return best;
        }
    }

    return best;
}

FlightAssignment find_two_opt_local_feasible_flight(
    const Instance &instance,
    const Solution &solution,
    const RouteTiming &timing,
    const std::set<Interval> &drone_intervals,
    const AffectedDroneFlight &affected,
    int first,
    int second)
{
    FlightAssignment best;
    const int route_size = (int)(solution.truck_route.size());
    if (route_size < 3)
    {
        return best;
    }

    const int segment_start = first + 1;
    const int segment_end = second;
    const int corridor_padding = 2;
    const int corridor_start = std::max(
        0,
        std::min(affected.launch_idx, segment_start) - corridor_padding);
    const int corridor_end = std::min(
        route_size - 2,
        std::max(affected.land_idx, segment_end) + corridor_padding);

    for (int launch_idx = corridor_start; launch_idx <= corridor_end; ++launch_idx)
    {
        for (int land_idx = launch_idx + 1; land_idx <= corridor_end; ++land_idx)
        {
            consider_flight_assignment(
                instance,
                solution,
                timing,
                drone_intervals,
                affected.delivery,
                affected.drone,
                launch_idx,
                land_idx,
                best);
        }
    }

    if (best.feasible)
    {
        return best;
    }

    return find_vicinity_feasible_flight(
        instance,
        solution,
        timing,
        drone_intervals,
        affected.delivery,
        affected.drone);
}

void append_drone_flight(
    Solution &solution,
    int drone,
    int delivery,
    const FlightAssignment &assignment)
{
    solution.drones[drone].launch_indices.push_back(assignment.launch_idx);
    solution.drones[drone].deliver_nodes.push_back(delivery);
    solution.drones[drone].land_indices.push_back(assignment.land_idx);
}
