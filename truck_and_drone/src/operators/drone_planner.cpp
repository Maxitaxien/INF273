#include "operators/drone_planner.h"
#include "general/get_customer_positions.h"
#include "general/get_not_covered_by_truck.h"
#include "general/random.h"
#include "general/sort_drone_collection.h"
#include "operators/route_timing.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <algorithm>
#include <unordered_map>
#include <vector>

namespace
{
long long flight_duration(
    const Instance &inst,
    int customer,
    const Flight &flight)
{
    return inst.drone_matrix[flight.first][customer] +
        inst.drone_matrix[customer][flight.second];
}

int flight_span(
    const Flight &flight,
    const std::unordered_map<int, int> &customer_positions)
{
    return customer_positions.at(flight.second) - customer_positions.at(flight.first);
}

void trim_candidate_flights(
    const Instance &inst,
    int customer,
    const std::unordered_map<int, int> &customer_positions,
    int max_flights_per_customer,
    std::vector<Flight> &candidates)
{
    if (max_flights_per_customer <= 0 ||
        (int)(candidates.size()) <= max_flights_per_customer)
    {
        return;
    }

    std::sort(
        candidates.begin(),
        candidates.end(),
        [&](const Flight &lhs, const Flight &rhs) {
            const long long lhs_duration = flight_duration(inst, customer, lhs);
            const long long rhs_duration = flight_duration(inst, customer, rhs);
            if (lhs_duration != rhs_duration)
            {
                return lhs_duration < rhs_duration;
            }

            const int lhs_span = flight_span(lhs, customer_positions);
            const int rhs_span = flight_span(rhs, customer_positions);
            if (lhs_span != rhs_span)
            {
                return lhs_span < rhs_span;
            }

            return lhs < rhs;
        });

    candidates.resize(max_flights_per_customer);
}

bool flights_overlap(
    const Flight &lhs,
    const Flight &rhs,
    const std::unordered_map<int, int> &customer_positions)
{
    int lhs_launch = customer_positions.at(lhs.first);
    int lhs_land = customer_positions.at(lhs.second);
    int rhs_launch = customer_positions.at(rhs.first);
    int rhs_land = customer_positions.at(rhs.second);

    if (lhs_launch > lhs_land)
    {
        std::swap(lhs_launch, lhs_land);
    }
    if (rhs_launch > rhs_land)
    {
        std::swap(rhs_launch, rhs_land);
    }

    return std::max(lhs_launch, rhs_launch) < std::min(lhs_land, rhs_land);
}

void clear_drone_schedule(DroneCollection &drone)
{
    drone.launch_indices.clear();
    drone.land_indices.clear();
}

void update_p(
    P &p_j,
    int assigned_customer,
    const Flight &new_flight,
    const std::unordered_map<int, int> &customer_positions)
{
    auto assigned_it = p_j.find(assigned_customer);
    if (assigned_it != p_j.end())
    {
        assigned_it->second = {new_flight};
    }

    for (auto &[customer, flights] : p_j)
    {
        if (customer == assigned_customer)
        {
            continue;
        }

        flights.erase(
            std::remove_if(
                flights.begin(),
                flights.end(),
                [&](const Flight &candidate) {
                    return flights_overlap(candidate, new_flight, customer_positions);
                }),
            flights.end());
    }
}

bool flight_feasible_with_timing(
    const Instance &inst,
    const Solution &solution,
    const RouteTiming &timing,
    int drone,
    int customer,
    const Flight &flight,
    const std::unordered_map<int, int> &customer_positions)
{
    const auto launch_it = customer_positions.find(flight.first);
    const auto land_it = customer_positions.find(flight.second);
    if (launch_it == customer_positions.end() || land_it == customer_positions.end())
    {
        return false;
    }

    const int launch_idx = launch_it->second;
    const int land_idx = land_it->second;
    if (launch_idx >= land_idx || land_idx >= (int)solution.truck_route.size() - 1)
    {
        return false;
    }

    const long long launch_time = std::max(
        timing.truck_arrival[launch_idx],
        timing.drone_ready_at_stop[drone][launch_idx]);
    const long long out_time = inst.drone_matrix[flight.first][customer];
    const long long back_time = inst.drone_matrix[customer][flight.second];
    const long long drone_return = launch_time + out_time + back_time;
    const long long drone_wait = std::max(
        timing.truck_arrival[land_idx] - drone_return,
        0LL);

    return out_time + back_time + drone_wait <= inst.lim;
}

P build_p_impl(
    const Instance &inst,
    const Solution &curr_sol,
    const std::vector<int> &drone_customers,
    int max_flights_per_customer)
{
    P result;
    const std::vector<int> &route = curr_sol.truck_route;
    const std::unordered_map<int, int> customer_positions = get_customer_positions(curr_sol);

    result.reserve(drone_customers.size());

    for (int drone_customer : drone_customers)
    {
        std::vector<Flight> &candidates = result[drone_customer];
        for (int launch_idx = 0; launch_idx < (int)(route.size()); ++launch_idx)
        {
            const int launch_node = route[launch_idx];
            for (int land_idx = launch_idx + 1; land_idx < (int)(route.size()); ++land_idx)
            {
                const int land_node = route[land_idx];
                const long long duration =
                    inst.drone_matrix[launch_node][drone_customer] +
                    inst.drone_matrix[drone_customer][land_node];

                if (duration <= inst.lim)
                {
                    candidates.emplace_back(launch_node, land_node);
                }
            }
        }

        trim_candidate_flights(
            inst,
            drone_customer,
            customer_positions,
            max_flights_per_customer,
            candidates);
    }

    return result;
}
}

P build_p(const Instance &inst, const Solution &curr_sol)
{
    return build_p_impl(inst, curr_sol, get_not_covered_by_truck(curr_sol), 0);
}

std::pair<long long, Solution> drone_planner(
    const Instance &inst,
    const Solution &curr_sol)
{
    return drone_planner(inst, curr_sol, 10, 0);
}

std::pair<long long, Solution> drone_planner(
    const Instance &inst,
    const Solution &curr_sol,
    int iterations,
    int max_flights_per_customer)
{
    return drone_planner(
        inst,
        curr_sol,
        iterations,
        max_flights_per_customer,
        -1);
}

std::pair<long long, Solution> drone_planner(
    const Instance &inst,
    const Solution &curr_sol,
    int iterations,
    int max_flights_per_customer,
    int drone_to_replan)
{
    long long best = objective_function_impl(inst, curr_sol);
    Solution best_sol = curr_sol;

    const int amnt_iter = std::max(1, iterations);
    const std::unordered_map<int, int> customer_positions = get_customer_positions(curr_sol);
    const std::vector<int> planned_customers =
        drone_to_replan >= 0
        ? curr_sol.drones[drone_to_replan].deliver_nodes
        : get_not_covered_by_truck(curr_sol);
    const P base_p = build_p_impl(
        inst,
        curr_sol,
        planned_customers,
        max_flights_per_customer);

    for (int iter = 0; iter < amnt_iter; ++iter)
    {
        Solution curr = curr_sol;
        bool iteration_valid = true;

        for (int d = 0; d < (int)(curr.drones.size()) && iteration_valid; ++d)
        {
            if (drone_to_replan >= 0 && d != drone_to_replan)
            {
                continue;
            }

            const std::vector<int> &customers = curr_sol.drones[d].deliver_nodes;
            if (customers.empty())
            {
                continue;
            }

            clear_drone_schedule(curr.drones[d]);

            P drone_candidates;
            drone_candidates.reserve(customers.size());
            for (int customer : customers)
            {
                auto it = base_p.find(customer);
                if (it != base_p.end())
                {
                    drone_candidates.emplace(customer, it->second);
                }
            }

            curr.drones[d].launch_indices.reserve(customers.size());
            curr.drones[d].land_indices.reserve(customers.size());

            for (int customer : customers)
            {
                auto it = drone_candidates.find(customer);
                if (it == drone_candidates.end() || it->second.empty())
                {
                    iteration_valid = false;
                    break;
                }

                const RouteTiming timing = compute_route_timing(inst, curr);
                std::vector<Flight> feasible_flights;
                feasible_flights.reserve(it->second.size());
                for (const Flight &candidate : it->second)
                {
                    if (flight_feasible_with_timing(
                            inst,
                            curr,
                            timing,
                            d,
                            customer,
                            candidate,
                            customer_positions))
                    {
                        feasible_flights.push_back(candidate);
                    }
                }

                if (feasible_flights.empty())
                {
                    iteration_valid = false;
                    break;
                }

                const Flight new_flight = get_random(feasible_flights);
                curr.drones[d].launch_indices.push_back(customer_positions.at(new_flight.first));
                curr.drones[d].land_indices.push_back(customer_positions.at(new_flight.second));
                update_p(drone_candidates, customer, new_flight, customer_positions);
            }

            if (customers.size() > 1)
            {
                sort_drone_collection(curr.drones[d]);
            }
        }

        if (!iteration_valid || !master_check(inst, curr, false))
        {
            continue;
        }

        const long long obj_val = objective_function_impl(inst, curr);
        if (obj_val < best)
        {
            best = obj_val;
            best_sol = curr;
        }
    }

    return {best, best_sol};
}

