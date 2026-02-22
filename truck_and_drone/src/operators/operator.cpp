#include "datahandling/instance.h"
#include "verification/solution.h"
#include "operators/one_reinsert.h"
#include "verification/feasibility_check.h"
#include <vector>
#include <random>

extern std::mt19937 gen;  // reuse global generator

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


Solution one_reinsert_random(const Instance& instance, const Solution& sol) {
    Solution neighbor = sol;

    // Random pop and insert
    std::uniform_int_distribution<int> pop_dist(1, 3);
    std::uniform_int_distribution<int> insert_dist(1, 3);

    int pop, insert;

    // Ensure valid pop/insert
    do {
        pop = pop_dist(gen);
        insert = insert_dist(gen);
    } while (
        (pop > 1 && (pop - 2 >= sol.drones.size() || sol.drones[pop - 2].deliver_nodes.empty())) ||
        (insert > 1 && insert - 2 >= sol.drones.size())
    );

    // Compute truck size after pop
    int truck_size_after_pop = sol.truck_route.size();
    if (pop == 1 && !sol.truck_route.empty()) {
        truck_size_after_pop--;
    }

    // Guard against invalid truck size
    if (truck_size_after_pop <= 1) {
        return sol; // fallback: return original
    }

    // Random insertion index
    std::uniform_int_distribution<int> idx_dist(1, truck_size_after_pop);
    int idx = idx_dist(gen);

    // Apply move
    neighbor = one_reinsert(instance, neighbor, pop, insert, idx);

    return neighbor;
}