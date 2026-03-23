#include "algorithms/gam.h"
#include "general/roulette_wheel_selection.h"
#include "verification/objective_value.h"
#include "verification/feasibility_check.h"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace
{
struct GAMStatistics
{
    int operator_failures = 0;
    int infeasible_candidates = 0;
    int accepted_moves = 0;
    int improving_accepts = 0;
    int non_improving_accepts = 0;
    int best_updates = 0;
};

struct GAMOperatorStatistics
{
    std::string name;
    double weight = 1.0;
    double segment_score = 0.0;
    double total_score = 0.0;
    int segment_uses = 0;
    int total_uses = 0;
    int accepted = 0;
    int improving_accepts = 0;
    int best_updates = 0;
    int failures = 0;
    int infeasible = 0;
};

std::vector<double> initialize_weights(
    const std::vector<NamedOperator> &ops,
    const std::vector<double> &initial_weights)
{
    std::vector<double> weights(ops.size(), 1.0);
    if (initial_weights.size() != ops.size())
    {
        return weights;
    }

    for (int i = 0; i < static_cast<int>(ops.size()); ++i)
    {
        if (initial_weights[i] > 0.0)
        {
            weights[i] = initial_weights[i];
        }
    }

    return weights;
}

std::vector<GAMOperatorStatistics> build_operator_statistics(
    const std::vector<NamedOperator> &ops,
    const std::vector<double> &weights)
{
    std::vector<GAMOperatorStatistics> result;
    result.reserve(ops.size());

    for (int i = 0; i < static_cast<int>(ops.size()); ++i)
    {
        const std::string fallback_name = "Operator " + std::to_string(i);
        result.push_back(GAMOperatorStatistics{
            ops[i].name.empty() ? fallback_name : ops[i].name,
            weights[i]});
    }

    return result;
}

int select_operator_index(const std::vector<GAMOperatorStatistics> &operator_stats)
{
    std::vector<double> weights;
    weights.reserve(operator_stats.size());
    for (const GAMOperatorStatistics &stats : operator_stats)
    {
        weights.push_back(stats.weight);
    }

    return roulette_wheel_selection(weights);
}

double compute_operator_reward(bool accepted, bool improving, bool new_best)
{
    if (new_best)
    {
        return 5.0;
    }

    if (accepted && improving)
    {
        return 2.0;
    }

    if (accepted)
    {
        return 0.5;
    }

    return 0.0;
}

void update_operator_weights(std::vector<GAMOperatorStatistics> &operator_stats)
{
    constexpr double reaction_factor = 0.2;
    constexpr double minimum_weight = 0.1;

    for (GAMOperatorStatistics &stats : operator_stats)
    {
        if (stats.segment_uses > 0)
        {
            const double average_score = stats.segment_score / stats.segment_uses;
            const double target_weight = 1.0 + average_score;
            stats.weight = std::max(
                minimum_weight,
                (1.0 - reaction_factor) * stats.weight + reaction_factor * target_weight);
        }

        stats.segment_score = 0.0;
        stats.segment_uses = 0;
    }
}

void log_gam_progress(
    int iteration,
    long long incumbent_cost,
    long long best_cost,
    int non_improving_iterations,
    const GAMStatistics &stats)
{
    std::cout
        << "[GAM] iter=" << iteration
        << " incumbent=" << incumbent_cost
        << " best=" << best_cost
        << " stall=" << non_improving_iterations
        << " accepted=" << stats.accepted_moves
        << " improving=" << stats.improving_accepts
        << " sideways/worse=" << stats.non_improving_accepts
        << " op_fail=" << stats.operator_failures
        << " infeasible=" << stats.infeasible_candidates
        << " best_updates=" << stats.best_updates
        << '\n';
}

void log_operator_statistics(const std::vector<GAMOperatorStatistics> &operator_stats)
{
    std::cout << std::fixed << std::setprecision(3);
    for (const GAMOperatorStatistics &stats : operator_stats)
    {
        std::cout
            << "[GAM][OP] name=\"" << stats.name << "\""
            << " weight=" << stats.weight
            << " total_score=" << stats.total_score
            << " uses=" << stats.total_uses
            << " accepted=" << stats.accepted
            << " improving=" << stats.improving_accepts
            << " best_updates=" << stats.best_updates
            << " failures=" << stats.failures
            << " infeasible=" << stats.infeasible
            << '\n';
    }
    std::cout << std::defaultfloat;
}

void log_gam_summary(
    long long incumbent_cost,
    long long best_cost,
    int non_improving_iterations,
    const GAMStatistics &stats,
    const std::vector<GAMOperatorStatistics> &operator_stats)
{
    std::cout
        << "[GAM] summary"
        << " incumbent=" << incumbent_cost
        << " best=" << best_cost
        << " stall=" << non_improving_iterations
        << " accepted=" << stats.accepted_moves
        << " improving=" << stats.improving_accepts
        << " sideways/worse=" << stats.non_improving_accepts
        << " op_fail=" << stats.operator_failures
        << " infeasible=" << stats.infeasible_candidates
        << " best_updates=" << stats.best_updates
        << '\n';

    log_operator_statistics(operator_stats);
}
}

Solution general_adaptive_metaheuristic(
    const Instance &instance,
    Solution initial,
    const std::vector<NamedOperator> &ops,
    const std::vector<double> &initial_weights)
{
    if (ops.empty())
    {
        return std::move(initial);
    }

    Solution incumbent = std::move(initial);
    long long incumbent_cost = objective_function_impl(instance, incumbent);
    Solution best = incumbent;
    long long best_cost = incumbent_cost;

    constexpr int max_iterations = 10000;
    constexpr int stopping_condition = 50;
    constexpr int segment_length = 100;
    constexpr int log_interval = 1000;

    int non_improving_iterations = 0;
    GAMStatistics stats;

    const std::vector<double> weights = initialize_weights(ops, initial_weights);
    std::vector<GAMOperatorStatistics> operator_stats =
        build_operator_statistics(ops, weights);

    for (int i = 0; i < max_iterations; ++i)
    {
        // TODO: Replace this with your actual escape/diversification phase.
        if (non_improving_iterations >= stopping_condition)
        {
            non_improving_iterations = 0;
        }

        const int selected_idx = select_operator_index(operator_stats);
        GAMOperatorStatistics &selected = operator_stats[selected_idx];
        selected.total_uses++;
        selected.segment_uses++;

        Solution neighbour = incumbent;
        if (!ops[selected_idx].op(instance, neighbour))
        {
            selected.failures++;
            stats.operator_failures++;
            non_improving_iterations++;
        }
        else if (!master_check(instance, neighbour, false))
        {
            selected.infeasible++;
            stats.infeasible_candidates++;
            non_improving_iterations++;
        }
        else
        {
            const long long cost = objective_function_impl(instance, neighbour);
            const long long delta_e = cost - incumbent_cost;

            bool accept = false;
            if (delta_e < 0)
            {
                accept = true;
                stats.improving_accepts++;
                selected.improving_accepts++;
            }
            else
            {
                // TODO: Replace this with your chosen acceptance rule.
                // With the current hill-climbing default, only improving moves are accepted.
                accept = false;
            }

            if (accept)
            {
                incumbent = neighbour;
                incumbent_cost = cost;
                stats.accepted_moves++;
                selected.accepted++;
                if (delta_e >= 0)
                {
                    stats.non_improving_accepts++;
                }
            }

            const bool new_best = cost < best_cost;
            if (new_best)
            {
                best = neighbour;
                best_cost = cost;
                stats.best_updates++;
                selected.best_updates++;
                non_improving_iterations = 0;
            }
            else
            {
                non_improving_iterations++;
            }

            const double reward = compute_operator_reward(accept, delta_e < 0, new_best);
            selected.segment_score += reward;
            selected.total_score += reward;
        }

        if ((i + 1) % segment_length == 0)
        {
            update_operator_weights(operator_stats);
            log_gam_progress(i + 1, incumbent_cost, best_cost, non_improving_iterations, stats);
            log_operator_statistics(operator_stats);
        }
        else if ((i + 1) % log_interval == 0)
        {
            log_gam_progress(i + 1, incumbent_cost, best_cost, non_improving_iterations, stats);
        }
    }

    log_gam_summary(incumbent_cost, best_cost, non_improving_iterations, stats, operator_stats);
    return best;
}
