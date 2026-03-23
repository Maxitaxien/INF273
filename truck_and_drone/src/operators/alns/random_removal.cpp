#pragma once
#include "operators/alns/random_removal.h"
#include "datahandling/instance.h"
#include "verification/solution.h"
#include "general/random.h"
#include <vector>

std::pair<bool, std::vector<int>> random_removal(const Instance &inst, Solution &sol, int n) {
    std::vector<int> result;
    for (int i = 0; i < n; i++) {
        // First: Choose whether to remove from truck, drone 1 or drone 2
        std::vector<int> values;
        int start = 0;
        bool from_drone = false;
        int segment;

        while (values.size() == 0) { // reselect if an empty segment is chosen
            segment = rand_int(0, 2);

            std::vector<int> values;

            if (segment == 0) {
                values = sol.truck_route;
                start = 1;
            } 
            else {
                values = sol.drones[segment - 1].deliver_nodes;
                from_drone = true;
            }
        }

        int position = rand_int(start, values.size() - 1);
        int value = values[position];
        values.erase(values.begin() + position);

        if (from_drone) {
            sol.drones[segment - 1].launch_indices.erase(sol.drones[segment - 1].launch_indices.begin() + position);
            sol.drones[segment - 1].land_indices.erase(sol.drones[segment - 1].land_indices.begin() + position);
        }

        result.push_back(value);
    }

    return {true, result};

}