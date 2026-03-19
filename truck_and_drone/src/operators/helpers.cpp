#include "operators/helpers.h"
#include "datahandling/instance.h"
#include "general/get_customer_positions.h"
#include "general/get_truck_arrival_times.h"
#include "verification/feasibility_check.h"
#include "verification/solution.h"
#include <algorithm>
#include <set>
#include <vector>

int pop_truck_delivery(Solution &solution, int i)
{
    int popped = solution.truck_route[i];
    solution.truck_route.erase(
        solution.truck_route.begin() + i);
    return popped;
}

void insert_truck_delivery(Solution &solution, int new_delivery, int i)
{
    solution.truck_route.insert(solution.truck_route.begin() + i, new_delivery);
}

void remove_drone_flight(Solution &solution, int drone, int i)
{
    solution.drones[drone].launch_indices.erase(
        solution.drones[drone].launch_indices.begin() + i);
    solution.drones[drone].land_indices.erase(
        solution.drones[drone].land_indices.begin() + i);
    solution.drones[drone].deliver_nodes.erase(
        solution.drones[drone].deliver_nodes.begin() + i);
}

void remove_drone_flight(Solution &solution, int drone)
{
    solution.drones[drone].launch_indices.pop_back();
    solution.drones[drone].land_indices.pop_back();
    solution.drones[drone].deliver_nodes.pop_back();
}

std::pair<bool, bool> drone_landed_at_back(const Solution &solution)
{
    int final_index = solution.truck_route.size() - 1;

    std::vector<bool> drone_is_invalid(2);

    for (int c = 0; c < solution.drones.size(); c++)
    {
        DroneCollection dc = solution.drones[c];
        for (int landing : dc.land_indices)
        {
            if (landing == final_index)
            {
                drone_is_invalid[c] = true;
            }
        }
    }
    return {drone_is_invalid[0], drone_is_invalid[1]};
}

std::vector<int> sort_by_distance_to_point_drone(const Instance &instance, const Solution &solution, int point)
{
    std::vector<int> points;
    const std::unordered_map<int, int> customer_positions = get_customer_positions(solution);

    for (int i = 1; i <= instance.n; i++)
    {
        if (i != point && customer_positions.find(i) != customer_positions.end())
        {
            points.push_back(i);
        }
    }

    // Sort indices by distance
    std::sort(points.begin(), points.end(),
              [&](int a, int b)
              {
                  return instance.drone_matrix[point][a] < instance.drone_matrix[point][b];
              });

    return points;
}

std::vector<int> sort_by_distance_to_point_truck(const Instance &instance, const Solution &solution, int point)
{
    std::vector<int> points;
    const std::unordered_map<int, int> customer_positions = get_customer_positions(solution);

    for (int i = 1; i <= instance.n; i++)
    {
        if (i != point && customer_positions.find(i) != customer_positions.end())
        {
            points.push_back(i);
        }
    }

    // Sort indices by distance
    std::sort(points.begin(), points.end(),
              [&](int a, int b)
              {
                  return instance.truck_matrix[point][a] < instance.truck_matrix[point][b];
              });

    return points;
}
