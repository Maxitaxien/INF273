#include "operators/route_timing.h"
#include <algorithm>
#include <utility>
#include <vector>

RouteTiming compute_route_timing(const Instance &instance, const Solution &solution)
{
    const int route_size = (int)(solution.truck_route.size());
    const int drone_count = (int)(solution.drones.size());

    RouteTiming timing;
    timing.truck_arrival.assign(route_size, 0);
    timing.drone_ready_at_stop.assign(
        drone_count,
        std::vector<long long>(route_size, 0));
    timing.drone_ready_at_end.assign(drone_count, 0);

    if (route_size == 0)
    {
        return timing;
    }

    std::vector<long long> truck_departure(route_size, 0);
    std::vector<long long> drone_ready(drone_count, 0);
    long long latest_drone_return_so_far = 0;

    std::vector<std::vector<std::pair<int, int>>> drone_returns_at(route_size);
    for (int d = 0; d < drone_count; ++d)
    {
        const DroneCollection &collection = solution.drones[d];
        const int flight_count = (int)(collection.launch_indices.size());
        for (int t = 0; t < flight_count; ++t)
        {
            const int land_idx = collection.land_indices[t];
            if (land_idx >= 0 && land_idx < route_size)
            {
                drone_returns_at[land_idx].emplace_back(d, t);
            }
        }
    }

    for (int d = 0; d < drone_count; ++d)
    {
        timing.drone_ready_at_stop[d][0] = 0;
    }

    for (int i = 1; i < route_size; ++i)
    {
        const int prev = solution.truck_route[i - 1];
        const int curr = solution.truck_route[i];

        timing.truck_arrival[i] =
            truck_departure[i - 1] + instance.truck_matrix[prev][curr];

        for (const auto &[drone, flight_idx] : drone_returns_at[i])
        {
            const DroneCollection &collection = solution.drones[drone];
            const int launch_idx = collection.launch_indices[flight_idx];
            const int deliver_node = collection.deliver_nodes[flight_idx];

            if (launch_idx < 0 || launch_idx >= route_size)
            {
                continue;
            }

            const int launch_node = solution.truck_route[launch_idx];
            const long long launch_time = std::max(
                timing.truck_arrival[launch_idx],
                timing.drone_ready_at_stop[drone][launch_idx]);
            const long long out_time =
                instance.drone_matrix[launch_node][deliver_node];
            const long long back_time =
                instance.drone_matrix[deliver_node][curr];
            const long long drone_arrival = launch_time + out_time;
            const long long drone_return = drone_arrival + back_time;

            timing.total_drone_arrival += drone_arrival;
            drone_ready[drone] = drone_return;
            latest_drone_return_so_far =
                std::max(latest_drone_return_so_far, drone_return);
        }

        for (int d = 0; d < drone_count; ++d)
        {
            timing.drone_ready_at_stop[d][i] = drone_ready[d];
        }

        truck_departure[i] =
            std::max(timing.truck_arrival[i], latest_drone_return_so_far);
    }

    timing.drone_ready_at_end = std::move(drone_ready);
    return timing;
}
