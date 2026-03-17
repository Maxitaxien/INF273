#pragma once
#include "operators/alns/signatures.h"
#include "datahandling/instance.h"
#include "verification/solution.h"
#include "operators/operator.h"

#include <vector>
#include <string>

namespace alns_heuristic
{
    inline const std::vector<RemovalHeuristic> removal = {
        worst_removal,
        random_removal};

    inline const std::vector<InsertionHeuristic> insertion = {
        greedy_insert};
}

/**
 * Adaptive ALNS-style composite operator.
 *
 * This object is stateful and callable, meaning it can be used wherever an
 * Operator (std::function<bool(...)>) is expected.
 *
 * Steps:
 * - select a removal operator
 * - select an insertion operator
 * - apply both
 * - track performance
 * - updates weights periodically
 */
class AdaptiveCompositeOperator
{
public:
    /**
     * Constructor
     *
     * @param removal_ops   List of destroy/remove operators
     * @param insertion_ops List of repair/insert operators
     * @param segment_length Number of iterations before weight update
     */
    AdaptiveCompositeOperator(
        std::vector<RemovalHeuristic> removal_ops,
        std::vector<InsertionHeuristic> insertion_ops,
        int segment_length = 100);

    /**
     * Callable interface — makes this usable as an Operator.
     *
     * Executes one ALNS iteration step.
     */
    bool operator()(const Instance &inst, Solution &sol);

private:
    // --- Operator sets ---
    std::vector<RemovalHeuristic> removal_ops_;
    std::vector<InsertionHeuristic> insertion_ops_;

    // --- Adaptive weights ---
    std::vector<double> removal_weights_;
    std::vector<double> insertion_weights_;

    // --- Score tracking (per segment) ---
    std::vector<double> removal_scores_;
    std::vector<double> insertion_scores_;

    std::vector<int> removal_usage_;
    std::vector<int> insertion_usage_;

    // --- Iteration tracking ---
    int iteration_ = 0;
    int segment_length_;

    // --- Last selected operators (for scoring) ---
    int last_removal_ = -1;
    int last_insertion_ = -1;

    // --- Adaptie neighbourhood size ---
    int n = 1;

private:
    // --- Internal helper methods ---

    /**
     * Select index using roulette wheel selection.
     */
    int select_removal_operator() const;
    int select_insertion_operator() const;

    /**
     * Assign score to last used operators.
     *
     * Call this AFTER evaluating solution quality.
     */
    void add_score(double score);

    /**
     * Update weights based on accumulated scores.
     * Called every segment_length_ iterations.
     */
    void update_weights();

    /**
     * Reset scores and usage counters after update.
     */
    void reset_segment_stats();
};