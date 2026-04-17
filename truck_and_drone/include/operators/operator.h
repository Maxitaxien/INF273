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
 * Targeted drone-to-truck reassignment for weak sorties.
 *
 * This ranks current drone flights by a cheap weakness signal, then tries a
 * small shortlist of candidates using a global best truck insertion for the
 * removed customer. The outer metaheuristic decides whether to accept the move.
 */
bool replace_drone_delivery_targeted(const Instance &instance, Solution &sol);

/**
 * Greedily move one or two drone deliveries back to the truck route.
 *
 * This is intended as a mild shake operator when the drone customer set has
 * become too static.
 */
bool drone_demotion_shake(const Instance &instance, Solution &sol);

/**
 * Proposal wrapper around local drone rendezvous shifts.
 *
 * This scans existing drone flights in random order, tries bounded launch/land
 * reassignments for one flight at a time, and commits the first feasible move
 * it finds. Acceptance is left to the outer metaheuristic.
 */
bool drone_rendezvous_shift_first_improvement(const Instance &instance, Solution &sol);

/**
 * Best-improvement wrapper around local drone rendezvous shifts.
 *
 * This evaluates the best bounded local shift for every existing drone flight and
 * commits the best feasible candidate across all of them. Acceptance is left
 * to the outer metaheuristic.
 */
bool drone_rendezvous_shift_best_improvement(const Instance &instance, Solution &sol);

/**
 * Swap two random customers in the combined Part 1 + Part 2 representation,
 * then repair the drone schedule with the planner if needed.
 *
 * @return Whether the sampled move yielded a feasible repaired candidate.
 */
bool two_opt_random(const Instance &inst, Solution &sol);

/**
 * Perform greedy 2-opt using truck gain as a screen and the repaired full
 * objective as the acceptance criterion.
 */
bool two_opt_greedy(const Instance &inst, Solution &sol);

/**
 * First-improvement 2-opt using the same truck-gain screen and repaired
 * full-objective acceptance as the greedy version.
 *
 * Scans edge pairs in order and commits the first improving feasible reversal.
 */
bool two_opt_first_improvement(const Instance &inst, Solution &sol);

/**
 * 2-opt ranked by truck-arrival surrogate, trying only the top few candidates.
 *
 * The first feasible screened move is returned and acceptance is left to the
 * outer metaheuristic.
 */
bool two_opt_arrival_screened(const Instance &inst, Solution &sol);

/**
 * Permute three random customers in the combined Part 1 + Part 2
 * representation, then repair the drone schedule with the planner if needed.
 *
 * @return Whether any sampled permutation improved the truck-route surrogate.
 */
bool three_opt_random(const Instance &inst, Solution &sol);

/**
 * Perform nearest neighbour reassignment from a random index. Then swaps the chosen one within the route.
 *
 * @return Whether the sampled move remained feasible.
 */
bool nearest_neighbour_reassign_random(const Instance &inst, Solution &sol);

/**
 * Reorder the current truck node set with Concorde's linkern heuristic.
 *
 * The truck cycle is optimized on the nodes currently assigned to the truck,
 * then the existing repair/planner path is used to rebuild any invalid drone
 * launch/land anchors. The move is kept only if the repaired full objective
 * improves.
 */
bool concorde_linkern_improve(const Instance &instance, Solution &sol);

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

/**
 * Replan a single randomly chosen non-empty drone route.
 *
 * This is a narrower planner-based shake than the full drone planner operator.
 * It commits any changed feasible one-drone replan and leaves acceptance to
 * the outer metaheuristic.
 */
bool single_drone_planner_shake(const Instance &instance, Solution &sol);

/**
 * Random Or-opt relocation on the combined Part 1 + Part 2 customer vector.
 */
bool or_opt_segment_relocate_random(const Instance &inst, Solution &sol);

/**
 * First-improvement Or-opt relocation on the combined Part 1 + Part 2
 * customer vector.
 */
bool or_opt_segment_relocate_first_improvement(const Instance &inst, Solution &sol);
