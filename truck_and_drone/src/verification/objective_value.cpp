#include "verification/objective_value.h"
#include <vector>
#include <algorithm>

long long calculate_total_waiting_time(
    const Instance& instance,
    const Solution& solution
) {
    long long total = 0;

    const auto& route = solution.truck_route;
    int m = route.size();

    // Truck arrival/departure times
    std::vector<long long> truck_arrival(m, 0);
    std::vector<long long> truck_departure(m, 0);

    // Drone availability times (one per drone)
    std::vector<long long> drone_available(solution.drones.size(), 0);

    // Precompute drone returns by truck index (land_index)
    std::vector<std::vector<const DroneTrip*>> drone_returns_at(m);

    for (int d = 0; d < solution.drones.size(); d++) {
        for (const auto& trip : solution.drones[d]) {
            if (trip.delivery_node != -1 && trip.land_index != -1) {
                drone_returns_at[trip.land_index].push_back(&trip);
            }
        }
    }

    // Main loop over truck stops
    for (int i = 1; i < m; i++) {
        int prev = route[i - 1];
        int curr = route[i];

        truck_arrival[i] = truck_departure[i - 1] + instance.truck_matrix[prev][curr];

        std::vector<long long> drone_returns_this_stop;

        for (const DroneTrip* trip : drone_returns_at[i]) {
            int launch_idx = trip->launch_index;
            int launch_node = trip->launch_node;
            int deliver_node = trip->delivery_node;

            long long out_time = instance.drone_matrix[launch_node][deliver_node];
            long long back_time = instance.drone_matrix[deliver_node][curr];

            // Drone can launch after truck arrives and after drone itself is available
            long long launch_time = std::max(truck_arrival[launch_idx], drone_available[0]);
            long long drone_arrival = launch_time + out_time;
            long long drone_return = drone_arrival + back_time;

            drone_available[0] = drone_return; // update availability
            drone_returns_this_stop.push_back(drone_return);

            total += drone_arrival; // add drone waiting
        }

        truck_departure[i] = drone_returns_this_stop.empty()
            ? truck_arrival[i]
            : std::max(truck_arrival[i], *std::max_element(drone_returns_this_stop.begin(), drone_returns_this_stop.end()));

        if (curr != 0)
            total += truck_arrival[i]; // add truck waiting
    }

    return total / 100;
}