#include "algorithms/gam.h"
#include "algorithms/gam_escape_algorithm.h"
#include "general/random.h"
#include "general/roulette_wheel_selection.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace
{
struct GAMOperatorState
{
    std::string name;
    double weight = 1.0;
    double segment_score = 0.0;
    int segment_uses = 0;
};

double average_absolute_delta_sample(
    const Instance &instance,
    const Solution &reference_solution,
    long long reference_cost,
    const std::vector<NamedOperator> &ops)
{
    constexpr int target_samples = 50;
    constexpr int max_attempts = 200;

    if (ops.empty())
    {
        return 1.0;
    }

    std::vector<long long> deltas;
    deltas.reserve(target_samples);

    for (int attempt = 0; attempt < max_attempts && (int)(deltas.size()) < target_samples; ++attempt)
    {
        const NamedOperator &op = ops[attempt % ops.size()];
        Solution neighbour = reference_solution;
        if (!op.op(instance, neighbour) || !master_check(instance, neighbour, false))
        {
            continue;
        }

        const long long candidate_cost = objective_function_impl(instance, neighbour);
        const long long delta = std::llabs(candidate_cost - reference_cost);
        if (delta > 0)
        {
            deltas.push_back(delta);
        }
    }

    if (deltas.empty())
    {
        return std::max(1.0, std::abs((double)reference_cost) * 0.01);
    }

    double total = 0.0;
    for (long long delta : deltas)
    {
        total += (double)delta;
    }

    return total / deltas.size();
}

double compute_acceptance_probability(
    long long incumbent_cost,
    long long candidate_cost,
    double temperature)
{
    if (temperature <= 0.0)
    {
        return 0.0;
    }

    const double delta = std::abs((double)(incumbent_cost - candidate_cost));
    return std::exp(-delta / temperature);
}

std::vector<double> initialize_weights(
    const std::vector<NamedOperator> &ops,
    const std::vector<double> &initial_weights)
{
    std::vector<double> weights(ops.size(), 1.0);
    if (initial_weights.size() != ops.size())
    {
        return weights;
    }

    for (int i = 0; i < (int)(ops.size()); ++i)
    {
        if (initial_weights[i] > 0.0)
        {
            weights[i] = initial_weights[i];
        }
    }

    return weights;
}

std::vector<GAMOperatorState> build_operator_state(
    const std::vector<NamedOperator> &ops,
    const std::vector<double> &weights)
{
    std::vector<GAMOperatorState> result;
    result.reserve(ops.size());

    for (int i = 0; i < (int)(ops.size()); ++i)
    {
        const std::string fallback_name = "Operator " + std::to_string(i);
        result.push_back(GAMOperatorState{
            ops[i].name.empty() ? fallback_name : ops[i].name,
            weights[i]});
    }

    return result;
}

std::vector<double> build_selection_weights(const std::vector<GAMOperatorState> &operator_state)
{
    std::vector<double> weights;
    weights.reserve(operator_state.size());

    for (const GAMOperatorState &state : operator_state)
    {
        weights.push_back(state.weight);
    }

    return weights;
}

double compute_operator_reward(
    bool improving,
    bool new_best)
{
    if (new_best)
    {
        return 4.0;
    }

    if (improving)
    {
        return 1.0;
    }

    return 0.0;
}

void update_operator_weights(
    std::vector<GAMOperatorState> &operator_state,
    std::vector<double> &selection_weights,
    int segment_index,
    int iteration,
    GAMRunStatistics &statistics)
{
    constexpr double reaction_factor = 0.2;

    for (int idx = 0; idx < (int)(operator_state.size()); ++idx)
    {
        GAMOperatorState &state = operator_state[idx];
        if (state.segment_uses > 0)
        {
            const double normalized_segment_score = state.segment_score / state.segment_uses;
            state.weight =
                state.weight * (1.0 - reaction_factor) +
                reaction_factor * normalized_segment_score;
        }

        selection_weights[idx] = state.weight;
        statistics.segment_stats.push_back(GAMSegmentStatistics{
            segment_index,
            iteration,
            idx,
            state.weight,
        });

        state.segment_score = 0.0;
        state.segment_uses = 0;
    }
}
}

GAMResult general_adaptive_metaheuristic(
    const Instance &instance,
    Solution initial,
    const std::vector<NamedOperator> &ops,
    const std::vector<double> &initial_weights)
{
    constexpr int max_iterations = 10000;
    constexpr int segment_length = 100;
    constexpr int stopping_condition = segment_length;

    GAMResult result;
    result.statistics.max_iterations = max_iterations;
    result.statistics.segment_length = segment_length;
    result.statistics.stopping_condition = stopping_condition;
    result.statistics.best_found_iteration = 0;
    result.statistics.iteration_stats.reserve(max_iterations);
    result.statistics.segment_stats.reserve(
        ((max_iterations + segment_length - 1) / segment_length) * ops.size());

    if (ops.empty())
    {
        result.solution = std::move(initial);
        return result;
    }

    const std::vector<double> initial_selection_weights = initialize_weights(ops, initial_weights);
    std::vector<GAMOperatorState> operator_state =
        build_operator_state(ops, initial_selection_weights);
    std::vector<double> selection_weights = build_selection_weights(operator_state);
    const double sampled_delta = average_absolute_delta_sample(
        instance,
        initial,
        objective_function_impl(instance, initial),
        ops);
    const double initial_temperature = -sampled_delta / std::log(0.8);
    const double final_temperature = std::max(1.0, initial_temperature * 0.01);
    const double cooling_rate = std::pow(
        final_temperature / initial_temperature,
        1.0 / std::max(1, max_iterations - 1));
    double temperature = initial_temperature;
    std::uniform_real_distribution<double> unit_dist(0.0, 1.0);

    result.statistics.operator_names.reserve(operator_state.size());
    for (const GAMOperatorState &state : operator_state)
    {
        result.statistics.operator_names.push_back(state.name);
    }

    Solution incumbent = std::move(initial);
    long long incumbent_cost = objective_function_impl(instance, incumbent);
    Solution best = incumbent;
    long long best_cost = incumbent_cost;
    int non_improving_iterations = 0;

    for (int i = 0; i < max_iterations; ++i)
    {
        if (non_improving_iterations >= stopping_condition)
        {
            incumbent = gam_escape_algorithm(instance, incumbent, ops, selection_weights, std::min(10, max_iterations - i));
            incumbent_cost = objective_function_impl(instance, incumbent);
            if (incumbent_cost < best_cost)
            {
                best = incumbent;
                best_cost = incumbent_cost;
                result.statistics.best_updates++;
                result.statistics.best_found_iteration = i;
            }
            non_improving_iterations = 0;
        }

        const int selected_idx = roulette_wheel_selection(selection_weights);
        GAMOperatorState &selected = operator_state[selected_idx];
        selected.segment_uses++;

        GAMIterationStatistics iteration_stat;
        iteration_stat.iteration = i + 1;
        iteration_stat.operator_idx = selected_idx;
        iteration_stat.temperature = temperature;

        Solution neighbour = incumbent;
        if (!ops[selected_idx].op(instance, neighbour))
        {
            result.statistics.operator_failures++;
            const double reward = compute_operator_reward(false, false);
            selected.segment_score += reward;
            non_improving_iterations++;
        }
        else if (!master_check(instance, neighbour, false))
        {
            result.statistics.infeasible_candidates++;
            const double reward = compute_operator_reward(false, false);
            selected.segment_score += reward;
            non_improving_iterations++;
        }
        else
        {
            const long long cost = objective_function_impl(instance, neighbour);
            const long long delta_e = cost - incumbent_cost;
            iteration_stat.delta = delta_e;
            iteration_stat.has_delta = true;

            bool accept = false;
            double acceptance_probability = 0.0;
            acceptance_probability = compute_acceptance_probability(
                incumbent_cost,
                cost,
                temperature);
            accept = unit_dist(gen) < acceptance_probability;
            if (delta_e > 0)
            {
                iteration_stat.worsening_acceptance_probability = acceptance_probability;
            }

            if (accept)
            {
                incumbent = neighbour;
                incumbent_cost = cost;
                result.statistics.accepted_moves++;
                if (delta_e < 0)
                {
                    result.statistics.improving_accepts++;
                }
                else
                {
                    result.statistics.non_improving_accepts++;
                }
            }

            const bool new_best = cost < best_cost;
            const bool incumbent_improved = accept && delta_e < 0;
            if (new_best)
            {
                best = neighbour;
                best_cost = cost;
                result.statistics.best_updates++;
                result.statistics.best_found_iteration = i + 1;
                non_improving_iterations = 0;
            }
            else if (incumbent_improved)
            {
                non_improving_iterations = 0;
            }
            else
            {
                non_improving_iterations++;
            }

            const double reward = compute_operator_reward(delta_e < 0, new_best);
            selected.segment_score += reward;
        }

        result.statistics.iteration_stats.push_back(iteration_stat);

        if ((i + 1) % segment_length == 0)
        {
            update_operator_weights(
                operator_state,
                selection_weights,
                (i + 1) / segment_length,
                i + 1,
                result.statistics);
        }

        temperature = std::max(final_temperature, temperature * cooling_rate);
    }

    if (max_iterations % segment_length != 0)
    {
        update_operator_weights(
            operator_state,
            selection_weights,
            max_iterations / segment_length + 1,
            max_iterations,
            result.statistics);
    }

    result.solution = std::move(best);
    return result;
}
