#include "operators/exact_segment_reopt.h"
#include "datahandling/instance_preprocessing.h"
#include "operators/operator.h"
#include "operators/route_timing.h"
#include "general/random.h"
#include "verification/objective_value.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <numeric>
#include <random>
#include <utility>
#include <vector>

namespace {

bool contains_node(const std::vector<int> &xs, int x)
{
    return std::find(xs.begin(), xs.end(), x) != xs.end();
}

void add_unique(std::vector<int> &xs, int x)
{
    if (!contains_node(xs, x))
        xs.push_back(x);
}

bool flight_touches_region(
    int launch_idx,
    int land_idx,
    int left_idx,
    int right_idx)
{
    return launch_idx >= left_idx && launch_idx <= right_idx
        || land_idx >= left_idx && land_idx <= right_idx
        || launch_idx < left_idx && land_idx > right_idx;
}

std::vector<int> collect_affected_customers(
    const Solution &sol,
    int start_idx,
    int k)
{
    const int left_idx = start_idx - 1;
    const int right_idx = start_idx + k;

    std::vector<int> affected;

    for (int idx = start_idx; idx < start_idx + k; ++idx)
        add_unique(affected, sol.truck_route[idx]);

    for (const DroneCollection &dc : sol.drones)
    {
        for (int t = 0; t < (int)dc.deliver_nodes.size(); ++t)
        {
            const int launch_idx = dc.launch_indices[t];
            const int land_idx = dc.land_indices[t];

            if (flight_touches_region(launch_idx, land_idx, left_idx, right_idx))
                add_unique(affected, dc.deliver_nodes[t]);
        }
    }

    return affected;
}

Solution remove_region_and_affected_flights(
    const Solution &sol,
    int start_idx,
    int k,
    int new_segment_size)
{
    const int left_idx = start_idx - 1;
    const int right_idx = start_idx + k;
    const int old_segment_size = k;
    const int delta = new_segment_size - old_segment_size;

    Solution base = sol;

    for (DroneCollection &dc : base.drones)
    {
        DroneCollection kept;

        for (int t = 0; t < (int)dc.deliver_nodes.size(); ++t)
        {
            int launch_idx = dc.launch_indices[t];
            int land_idx = dc.land_indices[t];
            const int customer = dc.deliver_nodes[t];

            if (flight_touches_region(launch_idx, land_idx, left_idx, right_idx))
                continue;

            if (launch_idx >= right_idx)
                launch_idx += delta;

            if (land_idx >= right_idx)
                land_idx += delta;

            kept.launch_indices.push_back(launch_idx);
            kept.deliver_nodes.push_back(customer);
            kept.land_indices.push_back(land_idx);
        }

        dc = std::move(kept);
    }

    return base;
}

void replace_segment(
    Solution &sol,
    int start_idx,
    int k,
    const std::vector<int> &new_segment)
{
    sol.truck_route.erase(
        sol.truck_route.begin() + start_idx,
        sol.truck_route.begin() + start_idx + k);

    sol.truck_route.insert(
        sol.truck_route.begin() + start_idx,
        new_segment.begin(),
        new_segment.end());
}

struct LocalFlight {
    int launch_local_idx;
    int customer;
    int land_local_idx;
};

using LocalFlightOptions = std::vector<std::vector<LocalFlight>>;

bool overlaps(const LocalFlight &a, const LocalFlight &b)
{
    return !(a.land_local_idx <= b.launch_local_idx ||
             b.land_local_idx <= a.launch_local_idx);
}

bool can_add_to_drone(
    const std::vector<LocalFlight> &flights,
    const LocalFlight &candidate)
{
    for (const LocalFlight &f : flights)
    {
        if (overlaps(f, candidate))
            return false;
    }

    return true;
}

long long objective_from_timing(const RouteTiming &timing)
{
    long long total_time = timing.total_drone_arrival;
    for (int idx = 1; idx < (int)timing.truck_arrival.size(); ++idx)
    {
        total_time += timing.truck_arrival[idx];
    }

    return total_time / 100;
}

bool canonical_drone_schedule_consistent(const Solution &solution)
{
    const int route_size = (int)solution.truck_route.size();

    for (const DroneCollection &collection : solution.drones)
    {
        if (collection.launch_indices.size() != collection.land_indices.size() ||
            collection.launch_indices.size() != collection.deliver_nodes.size())
        {
            return false;
        }

        std::vector<std::pair<int, int>> flights;
        flights.reserve(collection.launch_indices.size());
        int last_flight_idx = -1;

        for (int idx = 0; idx < (int)collection.launch_indices.size(); ++idx)
        {
            if (last_flight_idx < 0 ||
                std::pair{
                    collection.launch_indices[idx],
                    collection.land_indices[idx]} >
                    std::pair{
                        collection.launch_indices[last_flight_idx],
                        collection.land_indices[last_flight_idx]})
            {
                last_flight_idx = idx;
            }
        }

        for (int idx = 0; idx < (int)collection.launch_indices.size(); ++idx)
        {
            const int launch_idx = collection.launch_indices[idx];
            const int land_idx = collection.land_indices[idx];
            const bool terminal_depot =
                is_terminal_depot_landing(solution, land_idx);

            if (launch_idx < 0 || launch_idx >= route_size || land_idx < 0 ||
                land_idx > route_size)
            {
                return false;
            }

            if (land_idx <= launch_idx)
            {
                return false;
            }

            if (terminal_depot && idx != last_flight_idx)
            {
                return false;
            }

            flights.emplace_back(launch_idx, land_idx);
        }

        std::sort(
            flights.begin(),
            flights.end(),
            [](const std::pair<int, int> &lhs, const std::pair<int, int> &rhs) {
                return lhs.first < rhs.first;
            });

        for (int idx = 1; idx < (int)flights.size(); ++idx)
        {
            if (flights[idx].first < flights[idx - 1].second)
            {
                return false;
            }
        }
    }

    return true;
}

bool evaluate_candidate(
    const Instance &inst,
    Solution candidate,
    long long &cost,
    Solution &canonical_candidate)
{
    canonical_candidate =
        canonicalize_terminal_depot_landings(inst, std::move(candidate));

    if (!canonical_drone_schedule_consistent(canonical_candidate))
    {
        return false;
    }

    const RouteTiming timing = compute_route_timing_from_canonical_solution(
        inst,
        canonical_candidate);

    for (int drone = 0; drone < (int)canonical_candidate.drones.size(); ++drone)
    {
        const DroneCollection &collection = canonical_candidate.drones[drone];
        for (int flight_idx = 0; flight_idx < (int)collection.launch_indices.size(); ++flight_idx)
        {
            const int launch_idx = collection.launch_indices[flight_idx];
            const int land_idx = collection.land_indices[flight_idx];
            const int customer = collection.deliver_nodes[flight_idx];
            const bool terminal_depot =
                is_terminal_depot_landing(canonical_candidate, land_idx);

            if (customer <= 0 || customer > inst.n)
            {
                return false;
            }

            const int launch_node = canonical_candidate.truck_route[launch_idx];
            const int land_node =
                terminal_depot ? 0 : canonical_candidate.truck_route[land_idx];
            const long long launch_time = std::max(
                timing.truck_arrival[launch_idx],
                timing.drone_ready_at_stop[drone][launch_idx]);
            const long long out_time =
                inst.drone_matrix[launch_node][customer];
            const long long back_time =
                inst.drone_matrix[customer][land_node];
            const long long drone_return = launch_time + out_time + back_time;
            const long long drone_wait = terminal_depot
                ? 0LL
                : std::max(timing.truck_arrival[land_idx] - drone_return, 0LL);

            if (out_time + back_time + drone_wait > inst.lim)
            {
                return false;
            }
        }
    }

    cost = objective_from_timing(timing);
    return true;
}

void append_local_flights_to_solution(
    Solution &sol,
    int start_idx,
    const std::vector<std::vector<LocalFlight>> &assigned)
{
    if ((int)sol.drones.size() < (int)assigned.size())
        sol.drones.resize(assigned.size());

    for (int d = 0; d < (int)assigned.size(); ++d)
    {
        for (const LocalFlight &f : assigned[d])
        {
            sol.drones[d].launch_indices.push_back(start_idx - 1 + f.launch_local_idx);
            sol.drones[d].deliver_nodes.push_back(f.customer);
            sol.drones[d].land_indices.push_back(start_idx - 1 + f.land_local_idx);
        }
    }
}

LocalFlightOptions feasible_flight_options(
    const Instance &inst,
    const std::vector<int> &local_truck_nodes,
    const std::vector<int> &drone_customers)
{
    std::vector<std::vector<LocalFlight>> options(drone_customers.size());

    for (int c_idx = 0; c_idx < (int)drone_customers.size(); ++c_idx)
    {
        const int customer = drone_customers[c_idx];

        for (int launch_local = 0; launch_local < (int)local_truck_nodes.size(); ++launch_local)
        {
            for (int land_local = launch_local + 1;
                 land_local < (int)local_truck_nodes.size();
                 ++land_local)
            {
                const int launch_node = local_truck_nodes[launch_local];
                const int land_node = local_truck_nodes[land_local];

                if (pure_drone_flight_within_limit(
                        inst,
                        launch_node,
                        customer,
                        land_node))
                {
                    options[c_idx].push_back(
                        LocalFlight{launch_local, customer, land_local});
                }
            }
        }
    }

    return options;
}

void enumerate_drone_assignments(
    const LocalFlightOptions &options,
    int drone_count,
    const std::function<void(const std::vector<std::vector<LocalFlight>> &)> &callback)
{
    for (const auto &opts : options)
    {
        if (opts.empty())
            return;
    }

    std::vector<std::vector<LocalFlight>> assigned(drone_count);

    std::function<void(int)> dfs = [&](int customer_idx)
    {
        if (customer_idx == (int)options.size())
        {
            callback(assigned);
            return;
        }

        for (const LocalFlight &f : options[customer_idx])
        {
            for (int d = 0; d < drone_count; ++d)
            {
                if (!can_add_to_drone(assigned[d], f))
                    continue;

                assigned[d].push_back(f);
                dfs(customer_idx + 1);
                assigned[d].pop_back();
            }
        }
    };

    dfs(0);
}

} // namespace


bool exact_segment_reopt(
    const Instance &inst,
    Solution &sol,
    int start_idx,
    int k)
{
    const int route_size = (int)sol.truck_route.size();

    if (k <= 0)
        return false;

    if (start_idx <= 0)
        return false;

    if (start_idx + k > route_size)
        return false;

    const std::vector<int> affected =
        collect_affected_customers(sol, start_idx, k);

    if (affected.empty())
        return false;

    // Safety guard. Full exact reassignment grows very quickly.
    if ((int)affected.size() > 12)
        return false;

    const int left_node = sol.truck_route[start_idx - 1];
    const int right_idx = start_idx + k;
    const int right_node =
        right_idx == route_size ? 0 : sol.truck_route[right_idx];

    Solution best_solution = sol;
    long long best_cost = objective_function_impl(inst, sol);
    bool improved = false;

    const int a = (int)affected.size();
    const int masks = 1 << a;

    for (int truck_mask = 0; truck_mask < masks; ++truck_mask)
    {
        std::vector<int> truck_customers;
        std::vector<int> drone_customers;

        for (int bit = 0; bit < a; ++bit)
        {
            if (truck_mask & (1 << bit))
                truck_customers.push_back(affected[bit]);
            else
                drone_customers.push_back(affected[bit]);
        }

        std::sort(truck_customers.begin(), truck_customers.end());

        do
        {
            std::vector<int> local_truck_nodes;
            local_truck_nodes.push_back(left_node);

            for (int x : truck_customers)
                local_truck_nodes.push_back(x);

            local_truck_nodes.push_back(right_node);

            const int new_segment_size = (int)truck_customers.size();
            const LocalFlightOptions flight_options =
                feasible_flight_options(inst, local_truck_nodes, drone_customers);
            Solution candidate_base =
                remove_region_and_affected_flights(
                    sol,
                    start_idx,
                    k,
                    new_segment_size);
            replace_segment(candidate_base, start_idx, k, truck_customers);

            auto try_candidate =
                [&](const std::vector<std::vector<LocalFlight>> &assigned_flights)
            {
                Solution candidate = candidate_base;
                append_local_flights_to_solution(candidate, start_idx, assigned_flights);
                long long cost = 0;
                Solution canonical_candidate;
                if (!evaluate_candidate(
                        inst,
                        std::move(candidate),
                        cost,
                        canonical_candidate))
                {
                    return;
                }

                if (cost < best_cost)
                {
                    best_cost = cost;
                    best_solution = std::move(canonical_candidate);
                    improved = true;
                }
            };

            if (drone_customers.empty())
            {
                std::vector<std::vector<LocalFlight>> empty(inst.m);
                try_candidate(empty);
            }
            else
            {
                enumerate_drone_assignments(
                    flight_options,
                    inst.m,
                    try_candidate);
            }

        } while (std::next_permutation(
            truck_customers.begin(),
            truck_customers.end()));
    }

    if (!improved)
        return false;

    sol = std::move(best_solution);
    return true;
}


bool exact_segment_reopt_random(
    const Instance &inst,
    Solution &sol,
    int k)
{
    const int route_size = (int)sol.truck_route.size();

    if (k <= 0)
        return false;

    if (route_size <= k)
        return false;

    const int min_start_idx = 1;
    const int max_start_idx = route_size - k;

    if (max_start_idx < min_start_idx)
        return false;

    std::vector<int> start_indices(max_start_idx - min_start_idx + 1);
    std::iota(start_indices.begin(), start_indices.end(), min_start_idx);
    random_shuffle(start_indices);

    for (int start_idx : start_indices)
    {
        if (exact_segment_reopt(inst, sol, start_idx, k))
            return true;
    }

    return false;
}

bool exact_segment_reopt_random_small(const Instance &inst, Solution &sol) {
    return exact_segment_reopt_random(inst, sol, 2);
}


bool exact_segment_reopt_random_medium(const Instance &inst, Solution &sol) {
    return exact_segment_reopt_random(inst, sol, 3);
}

bool exact_segment_reopt_random_large(const Instance &inst, Solution &sol) {
    return exact_segment_reopt_random(inst, sol, 4);
}
