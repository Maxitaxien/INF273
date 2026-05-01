#include "internal.h"

#include "datahandling/instance_preprocessing.h"
#include "general/get_customer_positions.h"
#include "operators/helpers.h"
#include "verification/objective_value.h"
#include <algorithm>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace
{
std::pair<bool, Solution> assign_launch_and_land_n_lookahead_impl(
    const Instance &instance,
    Solution &solution,
    int idx,
    int new_deliver,
    int drone,
    int look_ahead,
    bool normalize_solution)
{
    if (normalize_solution)
    {
        simple_fix_validity(solution);
    }

    if (drone < 0 || drone >= (int)solution.drones.size() ||
        idx < 0 || idx >= (int)solution.truck_route.size() ||
        delivery_on_truck_route(solution, new_deliver))
    {
        return {false, solution};
    }

    const std::vector<int> &launches = solution.drones[drone].launch_indices;
    int maximum_lookahead = look_ahead;
    if (!launches.empty())
    {
        int i = 0;
        while (i < (int)launches.size() && launches[i] < idx)
        {
            ++i;
        }
        if (i < (int)launches.size())
        {
            maximum_lookahead = std::min(maximum_lookahead, launches[i] - idx);
        }
    }

    const int route_size = (int)solution.truck_route.size();
    const int max_offset = std::min(maximum_lookahead, route_size - idx - 1);
    if (max_offset <= 0)
    {
        return {false, solution};
    }

    const RouteTiming timing = compute_route_timing(instance, solution);
    const std::set<Interval> drone_intervals = get_intervals(solution, drone);
    FlightAssignment best;

    for (int offset = 1; offset <= max_offset; ++offset)
    {
        const int land_idx = idx + offset;
        if (land_idx >= route_size - 1 || overlaps(drone_intervals, idx, land_idx))
        {
            continue;
        }

        const int launch_node = solution.truck_route[idx];
        const int land_node = solution.truck_route[land_idx];
        if (!pure_drone_flight_within_limit(instance, launch_node, new_deliver, land_node))
        {
            continue;
        }

        const long long launch_time = std::max(
            timing.truck_arrival[idx],
            timing.drone_ready_at_stop[drone][idx]);
        const long long out_time = instance.drone_matrix[launch_node][new_deliver];
        const long long back_time = instance.drone_matrix[new_deliver][land_node];
        const long long drone_arrival = launch_time + out_time;
        const long long drone_return = drone_arrival + back_time;
        const long long drone_wait = std::max(
            timing.truck_arrival[land_idx] - drone_return,
            0LL);
        const long long total_with_wait = out_time + back_time + drone_wait;
        if (total_with_wait > instance.lim)
        {
            continue;
        }

        const long long truck_wait = std::max(
            drone_return - timing.truck_arrival[land_idx],
            0LL);
        const long long downstream_stops = std::max(0, route_size - land_idx - 1);
        const long long downstream_delay_impact = truck_wait * downstream_stops;
        if (!best.feasible ||
            downstream_delay_impact < best.downstream_delay_impact ||
            (downstream_delay_impact == best.downstream_delay_impact &&
             truck_wait < best.truck_wait) ||
            (downstream_delay_impact == best.downstream_delay_impact &&
             truck_wait == best.truck_wait &&
             drone_wait < best.drone_wait) ||
            (downstream_delay_impact == best.downstream_delay_impact &&
             truck_wait == best.truck_wait &&
             drone_wait == best.drone_wait &&
             land_idx < best.land_idx))
        {
            best.feasible = true;
            best.launch_idx = idx;
            best.land_idx = land_idx;
            best.downstream_delay_impact = downstream_delay_impact;
            best.truck_wait = truck_wait;
            best.drone_wait = drone_wait;
            best.drone_arrival = drone_arrival;
        }
    }

    if (!best.feasible)
    {
        return {false, solution};
    }

    append_drone_flight(solution, drone, new_deliver, best);
    return {true, solution};
}

std::pair<bool, Solution> greedy_assign_launch_and_land_impl(
    const Instance &instance,
    Solution &solution,
    int new_deliver,
    int drone,
    bool normalize_solution)
{
    if (normalize_solution)
    {
        simple_fix_validity(solution);
    }

    if (drone < 0 || drone >= (int)solution.drones.size() ||
        delivery_on_truck_route(solution, new_deliver))
    {
        return {false, solution};
    }

    const RouteTiming timing = compute_route_timing(instance, solution);
    const std::set<Interval> drone_intervals = get_intervals(solution, drone);
    const FlightAssignment best = find_vicinity_feasible_flight(
        instance,
        solution,
        timing,
        drone_intervals,
        new_deliver,
        drone);

    if (!best.feasible)
    {
        const int route_size = (int)(solution.truck_route.size());
        if (route_size < 3)
        {
            return {false, solution};
        }

        const std::unordered_map<int, int> customer_positions = get_customer_positions(solution);
        std::unordered_set<int> seen_launch_indices;
        const int look_ahead = std::min(
            route_size - 1,
            std::max(3, instance.n / 10));
        const int max_launch_candidates = std::min(4, route_size - 1);
        bool found_fallback = false;
        long long best_score = std::numeric_limits<long long>::max();
        Solution best_candidate = solution;

        for (int node : sort_by_distance_to_point_drone(instance, solution, new_deliver))
        {
            auto pos_it = customer_positions.find(node);
            if (pos_it == customer_positions.end())
            {
                continue;
            }

            const int launch_idx = pos_it->second;
            if (launch_idx < 0 || launch_idx >= route_size - 1 ||
                !seen_launch_indices.insert(launch_idx).second)
            {
                continue;
            }

            Solution candidate = solution;
            auto [assigned_ok, ignored_solution] = assign_launch_and_land_n_lookahead_impl(
                instance,
                candidate,
                launch_idx,
                new_deliver,
                drone,
                look_ahead,
                false);
            (void)ignored_solution;
            if (!assigned_ok)
            {
                if ((int)(seen_launch_indices.size()) >= max_launch_candidates)
                {
                    break;
                }
                continue;
            }

            const long long score = objective_function_impl(instance, candidate);
            if (!found_fallback || score < best_score)
            {
                found_fallback = true;
                best_score = score;
                best_candidate = std::move(candidate);
            }

            if ((int)(seen_launch_indices.size()) >= max_launch_candidates)
            {
                break;
            }
        }

        if (!found_fallback)
        {
            return {false, solution};
        }

        solution = std::move(best_candidate);
        return {true, solution};
    }

    append_drone_flight(solution, drone, new_deliver, best);
    return {true, solution};
}
}

std::pair<bool, Solution> assign_launch_and_land_n_lookahead(
    const Instance &instance,
    Solution &solution,
    int idx,
    int new_deliver,
    int drone,
    int look_ahead)
{
    return assign_launch_and_land_n_lookahead_impl(
        instance,
        solution,
        idx,
        new_deliver,
        drone,
        look_ahead,
        true);
}

std::pair<bool, Solution> assign_launch_and_land_n_lookahead_assume_valid(
    const Instance &instance,
    Solution &solution,
    int idx,
    int new_deliver,
    int drone,
    int look_ahead)
{
    return assign_launch_and_land_n_lookahead_impl(
        instance,
        solution,
        idx,
        new_deliver,
        drone,
        look_ahead,
        false);
}

std::pair<bool, Solution> greedy_assign_launch_and_land(
    const Instance &instance,
    Solution &solution,
    int new_deliver,
    int drone)
{
    return greedy_assign_launch_and_land_impl(
        instance,
        solution,
        new_deliver,
        drone,
        true);
}

std::pair<bool, Solution> greedy_assign_launch_and_land_assume_valid(
    const Instance &instance,
    Solution &solution,
    int new_deliver,
    int drone)
{
    return greedy_assign_launch_and_land_impl(
        instance,
        solution,
        new_deliver,
        drone,
        false);
}
