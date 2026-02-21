#include "datahandling/instance.h"
#include "verification/solution.h"
#include "operators/one_reinsert.h"
#include "verification/feasibility_check.h"
#include <vector>

std::vector<Solution> one_reinsert_operator(const Instance& instance, const Solution& sol) {
    std::vector<Solution> neighbors;

    for (int pop = 1; pop <= 3; pop++) {
        for (int insert = 1; insert <= 3; insert++) {
            Solution s = sol;

            // Determine valid truck size after pop
            int truck_size_after_pop = s.truck_route.size();
            if (pop == 1 && !s.truck_route.empty()) {
                truck_size_after_pop--; // last node will be popped
            }

            for (int idx = 1; idx <= truck_size_after_pop; ++idx) {
                Solution neighbor = s;
                
                // Skip if drone index would be invalid
                if (pop > 1 && (pop-2 >= neighbor.drones.size() || neighbor.drones[pop-2].deliver_nodes.empty()))
                    continue;
                if (insert > 1 && (insert-2 >= neighbor.drones.size()))
                    continue;

                neighbor = one_reinsert(instance, neighbor, pop, insert, idx);
                if (master_check(instance, neighbor, false)) {
                    neighbors.push_back(neighbor);
                }
            }
        }
    }

    return neighbors;
}