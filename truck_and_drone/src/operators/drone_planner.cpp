#include "operators/drone_planner.h"
#include "general/get_customer_positions.h"
#include "general/get_not_covered_by_truck.h"
#include "general/random.h"
#include "general/sort_drone_collection.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <algorithm>
#include <unordered_map>
#include <vector>

namespace
{
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
}

P build_p(const Instance &inst, const Solution &curr_sol)
{
    P result;
    const std::vector<int> drone_customers = get_not_covered_by_truck(curr_sol);
    const std::vector<int> &route = curr_sol.truck_route;

    result.reserve(drone_customers.size());

    for (int drone_customer : drone_customers)
    {
        std::vector<Flight> &candidates = result[drone_customer];
        for (int launch_idx = 0; launch_idx < static_cast<int>(route.size()); ++launch_idx)
        {
            const int launch_node = route[launch_idx];
            for (int land_idx = launch_idx + 1; land_idx < static_cast<int>(route.size()); ++land_idx)
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
    }

    return result;
}

std::pair<long long, Solution> drone_planner(
    const Instance &inst,
    const Solution &curr_sol)
{
    long long best = objective_function_impl(inst, curr_sol);
    Solution best_sol = curr_sol;

    const int amnt_iter = 10;
    const std::unordered_map<int, int> customer_positions = get_customer_positions(curr_sol);
    const P base_p = build_p(inst, curr_sol);

    for (int iter = 0; iter < amnt_iter; ++iter)
    {
        Solution curr = curr_sol;
        bool iteration_valid = true;

        for (DroneCollection &drone : curr.drones)
        {
            clear_drone_schedule(drone);
        }

        for (int d = 0; d < static_cast<int>(curr.drones.size()) && iteration_valid; ++d)
        {
            P drone_candidates = base_p;
            const std::vector<int> &customers = curr_sol.drones[d].deliver_nodes;

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

                const Flight new_flight = get_random(it->second);
                curr.drones[d].launch_indices.push_back(customer_positions.at(new_flight.first));
                curr.drones[d].land_indices.push_back(customer_positions.at(new_flight.second));
                update_p(drone_candidates, customer, new_flight, customer_positions);
            }

            sort_drone_collection(curr.drones[d]);
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
