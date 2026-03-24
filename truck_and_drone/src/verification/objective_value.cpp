#include "verification/objective_value.h"
#include "general/get_truck_arrival_times.h"
#include <vector>
#include <algorithm>
#include <map>


long long objective_function_impl(const Instance &instance, const Solution &solution)
{
    std::vector<long long> drone_available;
    long long total_drone_arrival = 0;

    // Compute truck arrival times and accumulate drone arrival times
    std::vector<long long> t_arrival = get_truck_arrival_times(
        instance, solution, drone_available, total_drone_arrival);

    long long total_time = total_drone_arrival;

    // Add truck arrival times (excluding depot)
    for (int i = 1; i < (int)(solution.truck_route.size()); ++i)
    {
        total_time += t_arrival[i];
    }

    // Scale units
    return total_time / 100;
}

long long objective_function_truck_only(const Instance &instance, const Solution &solution) {
    long long truck_objective = 0;
    for (int i = 0; i < solution.truck_route.size() - 1; i++) {
        int a = solution.truck_route[i];
        int b = solution.truck_route[i + 1];
        truck_objective += instance.truck_matrix[a][b];
    }
    return truck_objective;
}
