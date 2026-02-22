#include "datahandling/instance.h"
#include "verification/solution.h"
#include "operators/one_reinsert.h"
#include "verification/feasibility_check.h"
#include <vector>
#include <random>

extern std::mt19937 gen;  // reuse global generator

// Generate all valid neighbors (optional, may be expensive)
std::vector<Solution> one_reinsert_operator(const Instance& instance, const Solution& sol) {
    std::vector<Solution> neighbors;

    for (int pop = 1; pop <= 3; pop++) {
        for (int insert = 1; insert <= 3; insert++) {
            int truck_size_after_pop = sol.truck_route.size();
            if (pop == 1 && !sol.truck_route.empty()) truck_size_after_pop--;

            for (int idx = 1; idx <= truck_size_after_pop; ++idx) {
                // Skip invalid drone pop/insert
                if (pop > 1 && (pop - 2 >= sol.drones.size() || sol.drones[pop - 2].deliver_nodes.empty()))
                    continue;
                if (insert > 1 && (insert - 2 >= sol.drones.size()))
                    continue;

                Solution neighbor = sol;  // one copy per trial
                if (one_reinsert(instance, neighbor, pop, insert, idx)) {
                    if (master_check(instance, neighbor, false)) {
                        neighbors.push_back(neighbor);
                    }
                }
            }
        }
    }

    return neighbors;
}

// Generate a single random neighbor
bool one_reinsert_random(const Instance& instance, Solution& sol) {
    std::uniform_int_distribution<int> pop_dist(1, 3);
    std::uniform_int_distribution<int> insert_dist(1, 3);

    int pop, insert;

    do {
        pop = pop_dist(gen);
        insert = insert_dist(gen);
    } while (
        (pop > 1 && (pop - 2 >= sol.drones.size() || sol.drones[pop - 2].deliver_nodes.empty())) ||
        (insert > 1 && insert - 2 >= sol.drones.size())
    );

    int truck_size_after_pop = sol.truck_route.size();
    if (pop == 1 && !sol.truck_route.empty()) truck_size_after_pop--;

    if (truck_size_after_pop <= 1) return false; // cannot pop/insert

    std::uniform_int_distribution<int> idx_dist(1, truck_size_after_pop);
    int idx = idx_dist(gen);

    return one_reinsert(instance, sol, pop, insert, idx); // in-place
}