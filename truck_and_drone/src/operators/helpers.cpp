#include "operators/helpers.h"
#include "general/get_customer_positions.h"
#include <algorithm>
#include <vector>

int pop_truck_delivery(Solution &solution, int i)
{
    int popped = solution.truck_route[i];
    solution.truck_route.erase(solution.truck_route.begin() + i);
    return popped;
}

void insert_truck_delivery(Solution &solution, int new_delivery, int i)
{
    solution.truck_route.insert(solution.truck_route.begin() + i, new_delivery);
}

void shift_drone_indices_after_truck_insert(Solution &solution, int insert_idx)
{
    for (DroneCollection &drone_collection : solution.drones)
    {
        const int flight_count = (int)drone_collection.launch_indices.size();
        for (int i = 0; i < flight_count; ++i)
        {
            if (drone_collection.launch_indices[i] >= insert_idx)
            {
                ++drone_collection.launch_indices[i];
            }

            if (drone_collection.land_indices[i] >= insert_idx)
            {
                ++drone_collection.land_indices[i];
            }
        }
    }
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
    const int final_index = (int)(solution.truck_route.size()) - 1;
    std::pair<bool, bool> drone_is_invalid{false, false};

    const int drone_count = std::min(2, (int)(solution.drones.size()));
    for (int drone = 0; drone < drone_count; ++drone)
    {
        const DroneCollection &dc = solution.drones[drone];
        for (int landing : dc.land_indices)
        {
            if (landing == final_index)
            {
                if (drone == 0)
                {
                    drone_is_invalid.first = true;
                }
                else
                {
                    drone_is_invalid.second = true;
                }
                break;
            }
        }
    }
    return drone_is_invalid;
}

int count_drone_deliveries(const Solution &solution)
{
    int delivery_count = 0;
    for (const DroneCollection &drone : solution.drones)
    {
        delivery_count += (int)drone.deliver_nodes.size();
    }

    return delivery_count;
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

    std::sort(points.begin(), points.end(),
              [&](int a, int b)
              {
                  return instance.truck_matrix[point][a] < instance.truck_matrix[point][b];
              });

    return points;
}

