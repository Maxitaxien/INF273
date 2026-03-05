#include "operators/helpers.h"
#include "verification/solution.h"
#include "datahandling/instance.h"
#include "verification/feasibility_check.h"
#include "general/get_truck_arrival_times.h"
#include <vector>
#include <algorithm>
#include <unordered_set>
#include <set>
#include <unordered_map>

void remove_truck_delivery(Solution &solution, int i)
{
    solution.truck_route.erase(
        solution.truck_route.begin() + i);
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

std::unordered_map<int, int> get_node_positions(const Solution &solution)
{
    std::unordered_map<int, int> positions;
    for (int i = 0; i < solution.truck_route.size(); i++)
    {
        positions[solution.truck_route[i]] = i;
    }

    return positions;
}

std::vector<int> sort_by_distance_to_point(const Instance &instance, const Solution &solution, int point)
{
    std::vector<int> points;
    std::unordered_set<int> truck_set(solution.truck_route.begin(), solution.truck_route.end());

    for (int i = 0; i < instance.n; i++)
    {
        if (i != point && truck_set.find(i) != truck_set.end())
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