#include "algorithms/greedy_drone_cover.h"
#include <iostream>

// TODO: Fix bug involving trying to send the drone off before truck arrived 
Solution greedy_drone_cover(const Instance& problem_instance, Solution truck_solution) {
    Solution new_solution;
    int truck_solution_length = truck_solution.truck_route.size();
    int lim = problem_instance.lim;

    int i = 0;

    std::vector<int> new_truck_route;

    while (i < truck_solution_length - 2) {
        int a = truck_solution.truck_route[i];
        int b = truck_solution.truck_route[i + 1];
        int c = truck_solution.truck_route[i + 2];

        long long ab = problem_instance.drone_matrix[a][b];
        long long bc = problem_instance.drone_matrix[b][c];
        long long drone_time = ab + bc; 

        long long truck_arrival_at_c = problem_instance.truck_matrix[a][c];
        long long effective_drone_time = std::max(drone_time, truck_arrival_at_c);

        new_truck_route.push_back(a);

        if (effective_drone_time <= problem_instance.lim) {
            truck_solution.drone_map[a][0] = std::make_tuple(b, c);
            i += 2;
        }

        else {
            new_truck_route.push_back(b);
            i += 2;
        }
    }

    // Add final unincluded nodes
    while (i < truck_solution_length) {
        new_truck_route.push_back(truck_solution.truck_route[i]);
        ++i;
    }

    new_solution.truck_route = new_truck_route;
    new_solution.drone_map = truck_solution.drone_map;

    return new_solution;
}