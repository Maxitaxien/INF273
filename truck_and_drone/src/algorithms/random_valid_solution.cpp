#include "algorithms/random_valid_solution.h"

#include "algorithms/simple_initial_solution.h"
#include "general/get_not_covered_by_truck.h"
#include "general/random.h"
#include "verification/feasibility_check.h"

#include <algorithm>
#include <numeric>
#include <utility>
#include <vector>

namespace
{
constexpr int RANDOM_VALID_MAX_ATTEMPTS = 64;

bool random_solution_postconditions_hold(int n, const Solution &solution)
{
    if (!includes_all_nodes(n, solution, false))
    {
        return false;
    }

    const int route_size = (int)solution.truck_route.size();
    const int final_stored_index = route_size - 1;

    for (const DroneCollection &drone : solution.drones)
    {
        if (drone.launch_indices.size() != drone.deliver_nodes.size() ||
            drone.land_indices.size() != drone.deliver_nodes.size())
        {
            return false;
        }

        for (int flight = 0; flight < (int)drone.deliver_nodes.size(); ++flight)
        {
            const int launch_idx = drone.launch_indices[(size_t)flight];
            const int land_idx = drone.land_indices[(size_t)flight];

            if (launch_idx < 0 ||
                land_idx < 0 ||
                launch_idx >= route_size ||
                land_idx >= final_stored_index ||
                launch_idx >= land_idx)
            {
                return false;
            }

            if (flight > 0 && launch_idx < drone.land_indices[(size_t)flight - 1])
            {
                return false;
            }
        }
    }

    return true;
}

Solution build_random_valid_solution_attempt(int n)
{
    Solution solution;

    std::vector<int> nodes(n);
    std::iota(nodes.begin(), nodes.end(), 1);

    const int k_truck = rand_int((n / 2) + 1, n);
    const std::vector<int> truck_nodes = random_sample(nodes, (size_t)k_truck);

    solution.truck_route.push_back(0);
    solution.truck_route.insert(
        solution.truck_route.end(),
        truck_nodes.begin(),
        truck_nodes.end());
    solution.drones.resize(2);

    std::vector<int> remaining = get_not_covered_by_truck(n, solution);
    random_shuffle(remaining);

    const int remaining_count = (int)remaining.size();
    const int route_size = (int)solution.truck_route.size();
    const int max_per_drone = std::max(0, route_size - 2);

    int min_assign = std::max(0, remaining_count - max_per_drone);
    int max_assign = std::min(remaining_count, max_per_drone);
    if (min_assign > max_assign)
    {
        min_assign = max_assign = std::min(remaining_count, max_per_drone);
    }

    const int first_drone_count = rand_int(min_assign, max_assign);
    const auto [drone_one_nodes, drone_two_nodes] =
        random_partition(remaining, (size_t)first_drone_count);

    auto assign_drone = [&](int drone_idx, const std::vector<int> &deliveries) {
        int last_land = 0;
        const int delivery_count = (int)deliveries.size();

        for (int delivery_idx = 0; delivery_idx < delivery_count; ++delivery_idx)
        {
            const int remaining_deliveries = delivery_count - delivery_idx;
            const int max_land = route_size - remaining_deliveries - 1;
            const int max_launch = max_land - 1;
            const int launch_idx = rand_int(last_land, max_launch);
            const int land_idx = rand_int(launch_idx + 1, max_land);

            solution.drones[(size_t)drone_idx].launch_indices.push_back(launch_idx);
            solution.drones[(size_t)drone_idx].deliver_nodes.push_back(deliveries[(size_t)delivery_idx]);
            solution.drones[(size_t)drone_idx].land_indices.push_back(land_idx);
            last_land = land_idx;
        }
    };

    assign_drone(0, drone_one_nodes);
    assign_drone(1, drone_two_nodes);
    return solution;
}
} // namespace

Solution random_valid_solution(int n)
{
    for (int attempt = 0; attempt < RANDOM_VALID_MAX_ATTEMPTS; ++attempt)
    {
        Solution solution = build_random_valid_solution_attempt(n);
        if (random_solution_postconditions_hold(n, solution))
        {
            return solution;
        }
    }

    return simple_initial_solution(n);
}
