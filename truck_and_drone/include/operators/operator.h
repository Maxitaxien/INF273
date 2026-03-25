#pragma once
#include "datahandling/instance.h"
#include "operators/alns/signatures.h"
#include "verification/solution.h"
#include <functional>
#include <string>
#include <vector>

using Operator = std::function<bool(const Instance &, Solution &)>;

/**
 * Named operator wrapper.
 *
 * This allows us to store a human-readable name for an operator and use it
 * when generating output folders / CSV headers.
 */
struct NamedOperator
{
    std::string name;
    Operator op;
};

/**
 * ALNS-style operator: one destroy/removal followed by one repair/insertion.
 *
 * This keeps ALNS-specific parameters grouped together while still allowing us
 * to adapt the pair to the generic Operator interface used by the algorithms.
 */
struct ALNSOperator
{
    RemovalHeuristic removal;
    InsertionHeuristic insertion;
    int neighbourhood_size = 1;
    int regret_k = 0;
};

struct NamedALNSOperator
{
    std::string name;
    ALNSOperator op;
};

using FullNeighbourhoodOperator = std::function<std::vector<Solution>(const Instance &, const Solution &)>;

/**
 * Adapts an ALNS remove+insert pair to the generic Operator interface.
 *
 * The move is staged on a copy and only committed if both phases succeed.
 */
Operator make_alns_operator(const ALNSOperator &op);

/**
 * Convenience overload for creating a named Operator from an ALNS pair.
 */
NamedOperator make_named_alns_operator(const NamedALNSOperator &op);

/**
 * Materializes the Cartesian product of removal and insertion heuristics into
 * regular named operators when that is useful for selection/weighting.
 */
std::vector<NamedOperator> combine_alns_operator_pairs(
    const std::vector<NamedRemovalHeuristic> &removals,
    const std::vector<NamedInsertionHeuristic> &insertions,
    int neighbourhood_size = 1,
    int regret_k = 0);

/**
 * Generates all feasible one reinsert options.
 * Too slow for practical use.
 */
std::vector<Solution> one_reinsert_operator(const Instance &instance, const Solution &sol);

/**
 * Picks a random remove and reinsert candidate.
 */
bool one_reinsert_random(const Instance &instance, Solution &sol);

/**
 * Generate a one_reinsert candidate by first randomly selecting the candidate to remove.
 * Then evaluate all insertion candidates, greedily pick the best
 */
bool one_reinsert_greedy(const Instance &instance, Solution &sol);

/**
 * Randomly samples truck-to-drone replacement candidates and keeps the first
 * feasible move.
 */
bool replace_truck_delivery_random(const Instance &instance, Solution &sol);

/**
 * Then, evaluate all replacements within the truck route for both drones.
 * Perform the best.
 */
bool replace_truck_delivery_greedy(const Instance &instance, Solution &sol);

/**
 * Randomly samples a drone delivery and reinserts it into the truck route.
 */
bool replace_drone_delivery_random(const Instance &instance, Solution &sol);

/**
 * Evaluate all drone-to-truck replacements and keep the best feasible move.
 */
bool replace_drone_delivery_greedy(const Instance &instance, Solution &sol);

/**
 * Greedily move one or two drone deliveries back to the truck route.
 *
 * This is intended as a mild shake operator when the drone customer set has
 * become too static.
 */
bool drone_demotion_shake(const Instance &instance, Solution &sol);

/**
 * Perform 2-opt on random indexes except from depot.
 *
 * @return Whether the sampled move remained feasible.
 */
bool two_opt_random(const Instance &inst, Solution &sol);

/**
 * Perform greedy 2-opt using the best improving truck-edge exchange.
 */
bool two_opt_greedy(const Instance &inst, Solution &sol);

/**
 * Perform 3-opt on random breakpoints while keeping the depot fixed.
 *
 * @return Whether the sampled move improved the truck route surrogate.
 */
bool three_opt_random(const Instance &inst, Solution &sol);

/**
 * Perform nearest neighbour reassignment from a random index. Then swaps the chosen one within the route.
 *
 * @return Whether the sampled move remained feasible.
 */
bool nearest_neighbour_reassign_random(const Instance &inst, Solution &sol);

/**
 * Rebuild the current drone schedule with the planner and keep the result only
 * if it strictly improves the current objective.
 */
bool drone_planner_improve(const Instance &instance, Solution &sol);

/**
 * Budgeted planner operator for large instances.
 *
 * Uses fewer randomized rebuilds and a trimmed flight candidate set so the
 * planner can act as a cheap intensifier inside GAM.
 */
bool drone_planner_light_improve(const Instance &instance, Solution &sol);
