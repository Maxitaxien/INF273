#include "algorithms/greedy_drone_cover.h"
#include <vector>

Solution greedy_drone_cover(const Instance& inst, Solution base) {
    Solution new_solution;
    int truck_solution_length = base.truck_route.size();
    int lim = inst.lim;

    int i = 0;

    std::vector<int> new_truck_route;

    while (i < truck_solution_length - 2) {
        int a = base.truck_route[i];
        int b = base.truck_route[i + 1];
        int c = base.truck_route[i + 2];

        long long drone_time = inst.drone_matrix[a][b] + inst.drone_matrix[b][c];
        long long truck_travel_ac = inst.truck_matrix[a][c];

        long long effective_drone_time = std::max(drone_time, truck_travel_ac);

        new_truck_route.push_back(a);

        if (effective_drone_time < inst.truck_matrix[a][b] + inst.truck_matrix[b][c] && effective_drone_time < lim) {
            base.drone_map[a][0] = {b, c}; // we only use one drone
            i += 2;
        }

        else {
            new_truck_route.push_back(b);
            i += 2;
        }
    }

    // Add final unincluded nodes
    while (i < truck_solution_length) {
        new_truck_route.push_back(base.truck_route[i]);
        ++i;
    }

    new_solution.truck_route = new_truck_route;
    new_solution.drone_map = base.drone_map;

    return new_solution;
}