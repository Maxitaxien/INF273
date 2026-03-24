#include "general/get_truck_arrival_times.h"
#include "operators/route_timing.h"

std::vector<long long> get_truck_arrival_times(
    const Instance &instance,
    const Solution &solution,
    std::vector<long long> &drone_available,
    long long &total_drone_arrival
)
{
    const RouteTiming timing = compute_route_timing(instance, solution);
    drone_available = timing.drone_ready_at_end;
    total_drone_arrival = timing.total_drone_arrival;
    return timing.truck_arrival;
}
