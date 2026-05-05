#include "operators/candidate_evaluation.h"

#include "datahandling/instance_preprocessing.h"
#include "operators/route_timing.h"

#include <algorithm>
#include <utility>
#include <vector>

namespace
{
bool contains_int(const std::vector<int> &values, int value)
{
    return std::find(values.begin(), values.end(), value) != values.end();
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
} // namespace

void add_unique_int(std::vector<int> &values, int value)
{
    if (!contains_int(values, value))
    {
        values.push_back(value);
    }
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

            if (launch_idx < 0 || launch_idx >= route_size ||
                land_idx < 0 || land_idx > route_size)
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

        std::sort(flights.begin(), flights.end());
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

bool evaluate_candidate_with_timing(
    const Instance &instance,
    Solution candidate,
    long long &cost,
    Solution &canonical_candidate)
{
    canonical_candidate =
        canonicalize_terminal_depot_landings(instance, std::move(candidate));

    if (!canonical_drone_schedule_consistent(canonical_candidate))
    {
        return false;
    }

    const RouteTiming timing =
        compute_route_timing_from_canonical_solution(
            instance,
            canonical_candidate);

    for (int drone = 0; drone < (int)canonical_candidate.drones.size(); ++drone)
    {
        const DroneCollection &collection = canonical_candidate.drones[drone];
        for (int flight_idx = 0;
             flight_idx < (int)collection.launch_indices.size();
             ++flight_idx)
        {
            const int launch_idx = collection.launch_indices[flight_idx];
            const int land_idx = collection.land_indices[flight_idx];
            const int customer = collection.deliver_nodes[flight_idx];

            if (customer <= 0 || customer > instance.n)
            {
                return false;
            }

            const bool terminal_depot =
                is_terminal_depot_landing(canonical_candidate, land_idx);
            const int launch_node = canonical_candidate.truck_route[launch_idx];
            const int land_node =
                terminal_depot ? 0 : canonical_candidate.truck_route[land_idx];

            const long long launch_time = std::max(
                timing.truck_arrival[launch_idx],
                timing.drone_ready_at_stop[drone][launch_idx]);
            const long long out_time =
                instance.drone_matrix[launch_node][customer];
            const long long back_time =
                instance.drone_matrix[customer][land_node];
            const long long drone_return =
                launch_time + out_time + back_time;
            const long long drone_wait = terminal_depot
                ? 0LL
                : std::max(timing.truck_arrival[land_idx] - drone_return, 0LL);

            if (out_time + back_time + drone_wait > instance.lim)
            {
                return false;
            }
        }
    }

    cost = objective_from_timing(timing);
    return true;
}

void insert_truck_customer_and_shift_drones(
    Solution &solution,
    int insert_pos,
    int customer)
{
    solution.truck_route.insert(
        solution.truck_route.begin() + insert_pos,
        customer);

    for (DroneCollection &collection : solution.drones)
    {
        for (int &idx : collection.launch_indices)
        {
            if (idx >= insert_pos)
            {
                ++idx;
            }
        }

        for (int &idx : collection.land_indices)
        {
            if (idx >= insert_pos)
            {
                ++idx;
            }
        }
    }
}

void append_direct_drone_flight(
    Solution &solution,
    int drone,
    int launch_idx,
    int customer,
    int land_idx)
{
    if ((int)solution.drones.size() <= drone)
    {
        solution.drones.resize(drone + 1);
    }

    solution.drones[drone].launch_indices.push_back(launch_idx);
    solution.drones[drone].deliver_nodes.push_back(customer);
    solution.drones[drone].land_indices.push_back(land_idx);
}
