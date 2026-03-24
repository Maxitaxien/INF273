#include "operators/solution_fixers.h"
#include "operators/drone_planner.h"
#include "operators/helpers.h"
#include "operators/interval_helpers.h"
#include "operators/route_timing.h"
#include "verification/feasibility_check.h"
#include <algorithm>
#include <limits>
#include <set>
#include <unordered_set>
#include <vector>

namespace
{
struct FlightAssignment
{
    bool feasible = false;
    int launch_idx = -1;
    int land_idx = -1;
    long long truck_wait = std::numeric_limits<long long>::max();
    long long drone_arrival = std::numeric_limits<long long>::max();
};

bool delivery_on_truck_route(const Solution &solution, int delivery)
{
    return std::find(
               solution.truck_route.begin(),
               solution.truck_route.end(),
               delivery) != solution.truck_route.end();
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
        launch_idx >= route_size || land_idx >= route_size - 1 ||
        launch_idx >= land_idx)
    {
        return false;
    }

    const int launch_node = solution.truck_route[launch_idx];
    const int land_node = solution.truck_route[land_idx];
    const long long launch_time = std::max(
        timing.truck_arrival[launch_idx],
        timing.drone_ready_at_stop[drone][launch_idx]);
    const long long out_time = instance.drone_matrix[launch_node][deliver];
    const long long back_time = instance.drone_matrix[deliver][land_node];
    const long long drone_return = launch_time + out_time + back_time;
    const long long drone_wait = land_node == 0
        ? 0
        : std::max(timing.truck_arrival[land_idx] - drone_return, 0LL);

    return out_time + back_time + drone_wait <= instance.lim;
}

FlightAssignment find_best_feasible_flight(
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

    for (int launch_idx = 0; launch_idx < route_size - 1; ++launch_idx)
    {
        for (int land_idx = launch_idx + 1; land_idx < route_size - 1; ++land_idx)
        {
            if (overlaps(drone_intervals, launch_idx, land_idx))
            {
                continue;
            }

            const int launch_node = solution.truck_route[launch_idx];
            const int land_node = solution.truck_route[land_idx];
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
                continue;
            }

            const long long truck_wait = std::max(
                drone_return - timing.truck_arrival[land_idx],
                0LL);

            if (!best.feasible ||
                truck_wait < best.truck_wait ||
                (truck_wait == best.truck_wait &&
                 drone_arrival < best.drone_arrival) ||
                (truck_wait == best.truck_wait &&
                 drone_arrival == best.drone_arrival &&
                 land_idx < best.land_idx))
            {
                best.feasible = true;
                best.launch_idx = launch_idx;
                best.land_idx = land_idx;
                best.truck_wait = truck_wait;
                best.drone_arrival = drone_arrival;
            }
        }
    }

    return best;
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
}

std::pair<bool, Solution> greedy_assign_launch_and_land(
    const Instance &instance,
    Solution &solution,
    int new_deliver,
    int drone)
{
    simple_fix_validity(solution);

    if (drone < 0 || drone >= (int)solution.drones.size() ||
        delivery_on_truck_route(solution, new_deliver))
    {
        return {false, solution};
    }

    const RouteTiming timing = compute_route_timing(instance, solution);
    const std::set<Interval> drone_intervals = get_intervals(solution, drone);
    const FlightAssignment best = find_best_feasible_flight(
        instance,
        solution,
        timing,
        drone_intervals,
        new_deliver,
        drone);

    if (!best.feasible)
    {
        return {false, solution};
    }

    append_drone_flight(solution, drone, new_deliver, best);
    return {true, solution};
}

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
        auto [success, ignored_solution] = greedy_assign_launch_and_land(
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
        for (int i = 0; i < (int)(drone_collection.deliver_nodes.size());)
        {
            const int launch = drone_collection.launch_indices[i];
            const int landing = drone_collection.land_indices[i];
            const int delivery = drone_collection.deliver_nodes[i];
            const bool invalid_index =
                route_size < 2 ||
                launch < 0 || landing < 0 ||
                launch >= route_size || landing >= route_size ||
                launch >= landing || landing >= final_index;
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

Solution fix_overall_feasibility(const Instance &instance, Solution &solution)
{
    simple_fix_validity(solution);
    if (master_check(instance, solution, false))
    {
        return solution;
    }

    const auto [planner_score, planned_solution] = drone_planner(instance, solution);
    (void)planner_score;
    if (master_check(instance, planned_solution, false))
    {
        return planned_solution;
    }

    Solution fallback = solution;
    for (int drone = 0; drone < (int)fallback.drones.size(); ++drone)
    {
        fix_feasibility_for_drone(instance, fallback, drone);
    }
    simple_fix_validity(fallback);
    return fallback;
}

