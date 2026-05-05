#include "operators/exact_segment_reopt.h"
#include "datahandling/instance_preprocessing.h"
#include "operators/candidate_evaluation.h"
#include "operators/operator.h"
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
        add_unique_int(affected, sol.truck_route[idx]);

    for (const DroneCollection &dc : sol.drones)
    {
        for (int t = 0; t < (int)dc.deliver_nodes.size(); ++t)
        {
            const int launch_idx = dc.launch_indices[t];
            const int land_idx = dc.land_indices[t];

            if (flight_touches_region(launch_idx, land_idx, left_idx, right_idx))
                add_unique_int(affected, dc.deliver_nodes[t]);
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
            append_direct_drone_flight(
                sol,
                d,
                start_idx - 1 + f.launch_local_idx,
                f.customer,
                start_idx - 1 + f.land_local_idx);
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
                if (!evaluate_candidate_with_timing(
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
