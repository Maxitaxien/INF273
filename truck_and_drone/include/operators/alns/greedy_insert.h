#pragma once
#include "verification/solution.h"
#include "datahandling/instance.h"
#include <queue>

struct Candidate {
    Solution sol;
    long long score;
};

struct MaxHeapCompare {
    bool operator()(const Candidate& a, const Candidate& b) const {
        return a.score < b.score;  // larger score = higher priority at top
    }
};

/**
 * Helper function for generating all valid combinations and keeping the top ones.
 * 
 * We only keep the top ones, then select pseudo-randomly between them.
 * 
 * Truck insertion slots correspond to positions 1..truck_route.size().
 * Drone assignments correspond to dedicated slots after the truck slots.
 * 
 * @param n number of insertion targets (truck slots + drone assignments)
 * @param m number of values to insert
 * @param p amount of candidates to keep
 * @param current the vector of indices we are currently recursively building
 * @param sol solution 
 */
void generate_and_keep_top_p(
    int n,
    int m, 
    int p, 
    std::vector<int> &insert_positions,
    const std::vector<int> &to_insert,
    const Instance &inst, 
    const Solution &sol,
    std::priority_queue<Candidate, std::vector<Candidate>, MaxHeapCompare> &top_p_heap
);

/**
 * For each combination of values to insert: evaluate, pick best
 * Works similarly to one reinsert operator, but supports multiple insertions.
 *
 * Instead of deterministically picking best at each step, use roulette wheel
 * to introduce some variance.
 *
 * @param to_insert ordered vector of elements to insert
 * @param k always 0 (given as dummy to match signature, used in regret-k)
 */
bool greedy_insert(const Instance &inst, Solution &sol, std::vector<int> to_insert, int k);
