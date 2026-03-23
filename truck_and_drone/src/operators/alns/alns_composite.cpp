#include "operators/alns/alns_composite.h"
#include "general/roulette_wheel_selection.h"

#include <algorithm>
#include <utility>
#include <vector>

AdaptiveCompositeOperator::AdaptiveCompositeOperator(
    std::vector<RemovalHeuristic> removal_ops,
    std::vector<InsertionHeuristic> insertion_ops,
    int segment_length)
    : removal_ops_(std::move(removal_ops)),
      insertion_ops_(std::move(insertion_ops)),
      removal_weights_(removal_ops_.size(), 1.0),
      insertion_weights_(insertion_ops_.size(), 1.0),
      removal_scores_(removal_ops_.size(), 0.0),
      insertion_scores_(insertion_ops_.size(), 0.0),
      removal_usage_(removal_ops_.size(), 0),
      insertion_usage_(insertion_ops_.size(), 0),
      segment_length_(std::max(1, segment_length))
{
}

int AdaptiveCompositeOperator::select_removal_operator() const
{
    if (removal_ops_.empty())
    {
        return -1;
    }

    return roulette_wheel_selection(removal_weights_);
}

int AdaptiveCompositeOperator::select_insertion_operator() const
{
    if (insertion_ops_.empty())
    {
        return -1;
    }

    return roulette_wheel_selection(insertion_weights_);
}

void AdaptiveCompositeOperator::add_score(double score)
{
    if (last_removal_ >= 0)
    {
        removal_scores_[last_removal_] += score;
    }

    if (last_insertion_ >= 0)
    {
        insertion_scores_[last_insertion_] += score;
    }
}

void AdaptiveCompositeOperator::update_weights()
{
    for (int idx = 0; idx < static_cast<int>(removal_weights_.size()); ++idx)
    {
        if (removal_usage_[idx] == 0)
        {
            continue;
        }

        const double avg_score = removal_scores_[idx] / removal_usage_[idx];
        removal_weights_[idx] = std::max(0.1, avg_score);
    }

    for (int idx = 0; idx < static_cast<int>(insertion_weights_.size()); ++idx)
    {
        if (insertion_usage_[idx] == 0)
        {
            continue;
        }

        const double avg_score = insertion_scores_[idx] / insertion_usage_[idx];
        insertion_weights_[idx] = std::max(0.1, avg_score);
    }
}

void AdaptiveCompositeOperator::reset_segment_stats()
{
    std::fill(removal_scores_.begin(), removal_scores_.end(), 0.0);
    std::fill(insertion_scores_.begin(), insertion_scores_.end(), 0.0);
    std::fill(removal_usage_.begin(), removal_usage_.end(), 0);
    std::fill(insertion_usage_.begin(), insertion_usage_.end(), 0);
}

bool AdaptiveCompositeOperator::operator()(const Instance &inst, Solution &sol)
{
    if (removal_ops_.empty() || insertion_ops_.empty())
    {
        return false;
    }

    ++iteration_;

    last_removal_ = select_removal_operator();
    last_insertion_ = select_insertion_operator();
    if (last_removal_ < 0 || last_insertion_ < 0)
    {
        return false;
    }

    removal_usage_[last_removal_]++;
    insertion_usage_[last_insertion_]++;

    Solution candidate = sol;

    const auto [removed_ok, removed_nodes] =
        removal_ops_[last_removal_](inst, candidate, n);
    if (!removed_ok)
    {
        return false;
    }

    if (!insertion_ops_[last_insertion_](inst, candidate, removed_nodes, 0))
    {
        return false;
    }

    add_score(1.0);
    sol = std::move(candidate);

    if (iteration_ % segment_length_ == 0)
    {
        update_weights();
        reset_segment_stats();
    }

    return true;
}
