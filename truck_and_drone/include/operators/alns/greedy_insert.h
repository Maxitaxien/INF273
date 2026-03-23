#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"
#include <queue>
#include <vector>

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
 * Exhaustive greedy insert.
 *
 * Evaluates combinations of insertion targets for all removed nodes, keeps the
 * top few, and samples from them. This is expensive but useful as a stronger
 * intensification operator.
 */
bool greedy_insert(const Instance &inst, Solution &sol, std::vector<int> to_insert, int k);

/**
 * Cheapest feasible sequential insert.
 *
 * Repeats the following until all removed nodes are reinserted:
 * - evaluate all feasible single-node insertions for all remaining nodes
 * - commit the cheapest feasible insertion
 *
 * This is much cheaper than the exhaustive greedy insert while still using the
 * same repair model for truck/drone placement.
 */
bool cheapest_feasible_sequential_insert(
    const Instance &inst,
    Solution &sol,
    std::vector<int> to_insert,
    int k);
