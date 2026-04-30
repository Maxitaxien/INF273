#include "operators/replace_drone_delivery.h"
#include "operators/operator.h"
#include "general/roulette_wheel_selection.h"
#include "operators/helpers.h"
#include "operators/route_timing.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <algorithm>
#include <limits>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

extern std::mt19937 gen;

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

int count_drone_deliveries(const Solution &sol)
{
    int delivery_count = 0;
    for (const DroneCollection &drone : sol.drones)
    {
        delivery_count += (int)(drone.deliver_nodes.size());
    }

    return delivery_count;
}

struct DroneDemotionCandidate
{
    int drone = -1;
    int flight_idx = -1;
    int land_idx = -1;
    int span = std::numeric_limits<int>::max();
    long long truck_wait = 0;
    long long drone_duration = 0;
    long long downstream_delay_impact = 0;
};

bool replace_drone_delivery_with_bounds(
    const Instance &inst,
    Solution &sol,
    int drone,
    int flight_idx,
    int insert_start,
    int insert_end,
    long long *best_objective_out = nullptr)
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
    const int delivery = drone_collection.deliver_nodes[flight_idx];
    const int safe_insert_start = std::max(1, insert_start);
    const int safe_insert_end = std::min(route_size, insert_end);
    if (route_size < 2 || safe_insert_start > safe_insert_end)
    {
        return false;
    }

    bool found_candidate = false;
    long long best_objective = std::numeric_limits<long long>::max();
    Solution best_solution;
    Solution candidate = sol;

    for (int insert_idx = safe_insert_start; insert_idx <= safe_insert_end; ++insert_idx)
    {
        candidate = sol;
        remove_drone_flight(candidate, drone, flight_idx);
        candidate.truck_route.insert(candidate.truck_route.begin() + insert_idx, delivery);
        shift_drone_indices_after_truck_insert(candidate, insert_idx);

        if (!master_check(inst, candidate, false))
        {
            continue;
        }

        const long long objective = objective_function_impl(inst, candidate);
        if (!found_candidate || objective < best_objective)
        {
            found_candidate = true;
            best_objective = objective;
            best_solution = std::move(candidate);
        }
    }

    if (!found_candidate)
    {
        return false;
    }

    if (best_objective_out != nullptr)
    {
        *best_objective_out = best_objective;
    }

    sol = std::move(best_solution);
    return true;
}

std::vector<DroneDemotionCandidate> rank_drone_demotion_candidates(
    const Instance &inst,
    const Solution &sol)
{
    std::vector<DroneDemotionCandidate> candidates;
    const int route_size = (int)(sol.truck_route.size());
    if (route_size < 2)
    {
        return candidates;
    }

    const RouteTiming timing = compute_route_timing(inst, sol);
    for (int drone = 0; drone < (int)(sol.drones.size()); ++drone)
    {
        const DroneCollection &collection = sol.drones[drone];
        const int flight_count = (int)(collection.deliver_nodes.size());
        for (int flight_idx = 0; flight_idx < flight_count; ++flight_idx)
        {
            const int launch_idx = collection.launch_indices[flight_idx];
            const int land_idx = collection.land_indices[flight_idx];
            const int delivery = collection.deliver_nodes[flight_idx];
            if (launch_idx < 0 || land_idx < 0 ||
                launch_idx >= route_size || land_idx >= route_size ||
                launch_idx >= land_idx)
            {
                continue;
            }

            const int launch_node = sol.truck_route[launch_idx];
            const int land_node = sol.truck_route[land_idx];
            const long long launch_time = std::max(
                timing.truck_arrival[launch_idx],
                timing.drone_ready_at_stop[drone][launch_idx]);
            const long long out_time = inst.drone_matrix[launch_node][delivery];
            const long long back_time = inst.drone_matrix[delivery][land_node];
            const long long drone_return = launch_time + out_time + back_time;
            const long long truck_wait = std::max(
                drone_return - timing.truck_arrival[land_idx],
                0LL);
            const long long downstream_stops =
                std::max(0, route_size - land_idx - 1);

            candidates.push_back(DroneDemotionCandidate{
                drone,
                flight_idx,
                land_idx,
                land_idx - launch_idx,
                truck_wait,
                out_time + back_time,
                truck_wait * downstream_stops});
        }
    }

    std::sort(
        candidates.begin(),
        candidates.end(),
        [](const DroneDemotionCandidate &lhs, const DroneDemotionCandidate &rhs) {
            if (lhs.downstream_delay_impact != rhs.downstream_delay_impact)
            {
                return lhs.downstream_delay_impact > rhs.downstream_delay_impact;
            }
            if (lhs.truck_wait != rhs.truck_wait)
            {
                return lhs.truck_wait > rhs.truck_wait;
            }
            if (lhs.land_idx != rhs.land_idx)
            {
                return lhs.land_idx < rhs.land_idx;
            }
            if (lhs.drone_duration != rhs.drone_duration)
            {
                return lhs.drone_duration > rhs.drone_duration;
            }
            if (lhs.span != rhs.span)
            {
                return lhs.span < rhs.span;
            }
            if (lhs.drone != rhs.drone)
            {
                return lhs.drone < rhs.drone;
            }
            return lhs.flight_idx < rhs.flight_idx;
        });

    return candidates;
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
    (void)delivery;
    return replace_drone_delivery_with_bounds(
        inst,
        sol,
        drone,
        flight_idx,
        insert_start,
        insert_end);
}

bool replace_drone_delivery_random(const Instance &instance, Solution &sol)
{
    std::vector<std::pair<int, int>> candidates;
    for (int drone = 0; drone < (int)(sol.drones.size()); ++drone)
    {
        const int flight_count = (int)(sol.drones[drone].deliver_nodes.size());
        for (int flight_idx = 0; flight_idx < flight_count; ++flight_idx)
        {
            candidates.emplace_back(drone, flight_idx);
        }
    }

    if (candidates.empty())
    {
        return false;
    }

    std::shuffle(candidates.begin(), candidates.end(), gen);
    const int max_attempts = std::min(8, (int)(candidates.size()));
    Solution candidate = sol;

    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
        const auto [drone, flight_idx] = candidates[attempt];
        candidate = sol;
        if (replace_drone_delivery(instance, candidate, drone, flight_idx))
        {
            sol = std::move(candidate);
            return true;
        }
    }

    return false;
}

bool replace_drone_delivery_greedy(const Instance &instance, Solution &sol)
{
    bool success = false;
    long long best_cost = std::numeric_limits<long long>::max();
    Solution best_solution;
    Solution candidate = sol;

    for (int drone = 0; drone < (int)(sol.drones.size()); ++drone)
    {
        const int flight_count = (int)(sol.drones[drone].deliver_nodes.size());
        for (int flight_idx = 0; flight_idx < flight_count; ++flight_idx)
        {
            candidate = sol;
            if (!replace_drone_delivery(instance, candidate, drone, flight_idx))
            {
                continue;
            }

            const long long cost = objective_function_impl(instance, candidate);
            if (cost < best_cost)
            {
                best_cost = cost;
                best_solution = std::move(candidate);
                success = true;
            }
        }
    }

    if (success)
    {
        sol = std::move(best_solution);
    }

    return success;
}

bool replace_drone_delivery_targeted(const Instance &instance, Solution &sol)
{
    std::vector<DroneDemotionCandidate> ranked =
        rank_drone_demotion_candidates(instance, sol);
    if (ranked.empty())
    {
        return false;
    }

    const int shortlist = std::min(
        (int)(ranked.size()),
        instance.n >= 50 ? 3 : 4);

    std::vector<std::pair<long long, Solution>> feasible_candidates;
    feasible_candidates.reserve(shortlist);

    for (int idx = 0; idx < shortlist; ++idx)
    {
        const DroneDemotionCandidate &choice = ranked[idx];
        Solution candidate = sol;
        long long candidate_objective = 0;
        if (!replace_drone_delivery_with_bounds(
                instance,
                candidate,
                choice.drone,
                choice.flight_idx,
                1,
                (int)(sol.truck_route.size()),
                &candidate_objective))
        {
            continue;
        }

        feasible_candidates.emplace_back(candidate_objective, std::move(candidate));
    }

    if (feasible_candidates.empty())
    {
        return false;
    }

    std::sort(
        feasible_candidates.begin(),
        feasible_candidates.end(),
        [](const auto &lhs, const auto &rhs) {
            return lhs.first < rhs.first;
        });
    const int selected_idx = roulette_wheel_selection_exponential(
        (int)feasible_candidates.size());
    sol = std::move(feasible_candidates[(size_t)selected_idx].second);
    return true;
}

bool drone_demotion_shake(const Instance &instance, Solution &sol)
{
    const int delivery_count = count_drone_deliveries(sol);
    if (delivery_count == 0)
    {
        return false;
    }

    Solution candidate = sol;
    const int demotions_to_apply = delivery_count >= 20 ? 2 : 1;
    bool changed = false;

    for (int i = 0; i < demotions_to_apply; ++i)
    {
        if (!replace_drone_delivery_greedy(instance, candidate))
        {
            break;
        }

        changed = true;
    }

    if (!changed)
    {
        return false;
    }

    sol = std::move(candidate);
    return true;
}
