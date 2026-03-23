#pragma once
#include "operators/alns/signatures.h"
#include "operators/alns/greedy_insert.h"
#include "operators/alns/random_removal.h"
#include "datahandling/instance.h"
#include "verification/solution.h"
#include "operators/operator.h"

#include <string>
#include <vector>

namespace alns_heuristic
{
    inline const std::vector<RemovalHeuristic> removal = {
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
     * Constructor.
     *
     * @param removal_ops List of destroy/remove operators
     * @param insertion_ops List of repair/insert operators
     * @param segment_length Number of iterations before weight update
     */
    AdaptiveCompositeOperator(
        std::vector<RemovalHeuristic> removal_ops,
        std::vector<InsertionHeuristic> insertion_ops,
        int segment_length = 100);

    /**
     * Callable interface - makes this usable as an Operator.
     *
     * Executes one ALNS iteration step.
     */
    bool operator()(const Instance &inst, Solution &sol);

private:
    std::vector<RemovalHeuristic> removal_ops_;
    std::vector<InsertionHeuristic> insertion_ops_;

    std::vector<double> removal_weights_;
    std::vector<double> insertion_weights_;

    std::vector<double> removal_scores_;
    std::vector<double> insertion_scores_;

    std::vector<int> removal_usage_;
    std::vector<int> insertion_usage_;

    int iteration_ = 0;
    int segment_length_;

    int last_removal_ = -1;
    int last_insertion_ = -1;

    int n = 1;

    int select_removal_operator() const;
    int select_insertion_operator() const;
    void add_score(double score);
    void update_weights();
    void reset_segment_stats();
};
