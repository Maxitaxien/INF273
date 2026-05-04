#include "algorithms/gam.h"
#include "algorithms/gam_escape_algorithm.h"
#include "algorithms/gam_solution_cache.h"
#include "algorithms/nearest_neighbour.h"
#include "algorithms/greedy_drone_cover.h"
#include "general/random.h"
#include "general/roulette_wheel_selection.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <random>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{
bool same_solution(const Solution &lhs, const Solution &rhs)
{
    if (lhs.truck_route != rhs.truck_route || lhs.drones.size() != rhs.drones.size())
    {
        return false;
    }

    for (int drone = 0; drone < (int)(lhs.drones.size()); ++drone)
    {
        const DroneCollection &lhs_collection = lhs.drones[drone];
        const DroneCollection &rhs_collection = rhs.drones[drone];
        if (lhs_collection.launch_indices != rhs_collection.launch_indices ||
            lhs_collection.deliver_nodes != rhs_collection.deliver_nodes ||
            lhs_collection.land_indices != rhs_collection.land_indices)
        {
            return false;
        }
    }

    return true;
}

struct GAMOperatorState
{
    std::string name;
    double weight = 1.0;
    double initial_weight = 1.0;

    double segment_score = 0.0;
    double segment_improvement = 0.0;
    int segment_uses = 0;
    int segment_successes = 0;
    int segment_accepts = 0;
    int segment_improvements = 0;
    int segment_new_bests = 0;
};

double average_positive_delta_sample(
    const Instance &instance,
    const Solution &reference_solution,
    long long reference_cost,
    const std::vector<NamedOperator> &ops,
    GAMSolutionCache *cache)
{
    constexpr int target_samples = 50;
    constexpr int max_attempts = 250;

    if (ops.empty())
    {
        return 1.0;
    }

    std::vector<long long> deltas;
    deltas.reserve(target_samples);

    for (int attempt = 0; attempt < max_attempts && (int)(deltas.size()) < target_samples; ++attempt)
    {
        const NamedOperator &op = ops[(size_t)(attempt % ops.size())];
        Solution neighbour = reference_solution;
        if (!op.op(instance, neighbour) ||
            same_solution(neighbour, reference_solution))
        {
            continue;
        }

        const GAMSolutionEvaluation evaluation =
            evaluate_solution_with_cache(instance, neighbour, cache);
        if (!evaluation.feasible || !evaluation.objective_known)
        {
            continue;
        }

        const long long candidate_cost = evaluation.objective;
        const long long delta = candidate_cost - reference_cost;
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
    long long worsening_delta,
    double temperature)
{
    if (temperature <= 0.0)
    {
        return 0.0;
    }

    return std::exp(-(double)(worsening_delta) / temperature);
}

double compute_allowed_deviation(
    long long best_cost,
    double schedule_elapsed,
    double schedule_duration,
    double allowed_deviation_fraction)
{
    const double safe_duration = std::max(1e-9, schedule_duration);
    const double progress = std::clamp(
        schedule_elapsed / safe_duration,
        0.0,
        1.0);
    return std::max(0.0, allowed_deviation_fraction) *
        (1.0 - progress) *
        static_cast<double>(best_cost);
}

double build_phase_initial_temperature(
    const Instance &instance,
    const Solution &reference_solution,
    long long reference_cost,
    const std::vector<NamedOperator> &phase_ops,
    GAMSolutionCache *cache)
{
    const double sampled_positive_delta = average_positive_delta_sample(
        instance,
        reference_solution,
        reference_cost,
        phase_ops,
        cache);
    const double target_initial_worsening_acceptance =
        instance.n >= 50 ? 0.12 : 0.18;
    return -sampled_positive_delta / std::log(target_initial_worsening_acceptance);
}

GAMEscapeResult run_exchange_k_large_escape(
    const Instance &instance,
    Solution incumbent,
    int amnt_iter,
    GAMSolutionCache *cache)
{
    const GAMSolutionEvaluation initial_evaluation =
        evaluate_solution_with_cache(instance, incumbent, cache);
    const long long initial_cost =
        initial_evaluation.objective_known
            ? initial_evaluation.objective
            : objective_function_impl(instance, incumbent);

    if (cache != nullptr && !initial_evaluation.objective_known)
    {
        gam_cache_known_feasible_solution(*cache, instance, incumbent, initial_cost);
    }

    Solution best = incumbent;
    long long incumbent_cost = initial_cost;
    long long best_cost = initial_cost;
    bool found_new_best = false;

    for (int attempt = 0; attempt < amnt_iter; ++attempt)
    {
        Solution neighbour = incumbent;
        if (!exchange_k_large(instance, neighbour) || same_solution(neighbour, incumbent))
        {
            continue;
        }

        const GAMSolutionEvaluation evaluation =
            evaluate_solution_with_cache(instance, neighbour, cache);
        if (!evaluation.feasible || !evaluation.objective_known)
        {
            continue;
        }

        incumbent = std::move(neighbour);
        incumbent_cost = evaluation.objective;
        if (incumbent_cost < best_cost)
        {
            best = incumbent;
            best_cost = incumbent_cost;
            found_new_best = true;
        }
    }

    return GAMEscapeResult{
        std::move(incumbent),
        incumbent_cost,
        std::move(best),
        best_cost,
        found_new_best};
}

GAMEscapeResult run_configured_escape(
    const Instance &instance,
    Solution incumbent,
    const std::vector<NamedOperator> &ops,
    const std::vector<double> &selection_weights,
    int amnt_iter,
    GAMEscapeMode escape_mode,
    GAMSolutionCache *cache)
{
    switch (escape_mode)
    {
    case GAMEscapeMode::ExchangeKLarge:
        return run_exchange_k_large_escape(
            instance,
            std::move(incumbent),
            amnt_iter,
            cache);
    case GAMEscapeMode::LegacyGAM:
    default:
        return gam_escape_algorithm(
            instance,
            std::move(incumbent),
            ops,
            selection_weights,
            amnt_iter,
            cache);
    }
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
        if (initial_weights[(size_t)i] > 0.0)
        {
            weights[(size_t)i] = initial_weights[(size_t)i];
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
            ops[(size_t)i].name.empty() ? fallback_name : ops[(size_t)i].name,
            weights[(size_t)i],
            weights[(size_t)i]});
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
    long long delta_e,
    bool accepted,
    bool new_best,
    bool new_solution,
    double delta_scale)
{
    const double safe_scale = std::max(1.0, delta_scale);
    double reward = 0.0;
    constexpr double novel_solution_bonus = 0.05;

    if (delta_e < 0)
    {
        const double normalized_improvement = std::min(5.0, (double)(-delta_e) / safe_scale);
        reward += normalized_improvement;
    }

    if (accepted)
    {
        reward += 0.10;
    }

    if (new_best)
    {
        reward += 2.0;
    }

    if (new_solution)
    {
        reward += novel_solution_bonus;
    }

    return reward;
}

void update_operator_weights(
    std::vector<GAMOperatorState> &operator_state,
    std::vector<double> &selection_weights,
    int segment_index,
    int iteration,
    GAMRunStatistics &statistics)
{
    constexpr double reaction_factor = 0.30;
    constexpr double min_weight_ratio = 0.20;
    constexpr double max_weight_ratio = 3.50;
    constexpr double accepted_bonus = 0.15;
    constexpr double success_bonus = 0.10;
    constexpr double improvement_bonus = 0.25;
    constexpr double best_bonus = 0.40;
    constexpr double pull_to_prior = 0.05;

    double target_total_weight = 0.0;
    for (const GAMOperatorState &state : operator_state)
    {
        target_total_weight += state.initial_weight;
    }

    std::vector<double> performance(operator_state.size(), 0.0);
    double performance_sum = 0.0;
    int active_count = 0;

    for (int idx = 0; idx < (int)(operator_state.size()); ++idx)
    {
        const GAMOperatorState &state = operator_state[(size_t)idx];
        if (state.segment_uses <= 0)
        {
            continue;
        }

        const double uses = (double)state.segment_uses;
        const double score_per_use = state.segment_score / uses;
        const double improvement_per_use = state.segment_improvement / uses;
        const double accepted_rate = (double)state.segment_accepts / uses;
        const double success_rate = (double)state.segment_successes / uses;
        const double improvement_rate = (double)state.segment_improvements / uses;
        const double best_rate = (double)state.segment_new_bests / uses;

        performance[(size_t)idx] =
            score_per_use +
            0.35 * improvement_per_use +
            accepted_bonus * accepted_rate +
            success_bonus * success_rate +
            improvement_bonus * improvement_rate +
            best_bonus * best_rate;

        performance_sum += performance[(size_t)idx];
        active_count++;
    }

    const double mean_performance =
        active_count > 0 ? performance_sum / active_count : 0.0;

    double total_weight = 0.0;

    for (int idx = 0; idx < (int)(operator_state.size()); ++idx)
    {
        GAMOperatorState &state = operator_state[(size_t)idx];

        if (state.segment_uses > 0)
        {
            const double centered_signal = performance[(size_t)idx] - mean_performance;
            const double multiplicative_update = std::exp(reaction_factor * centered_signal);

            state.weight *= multiplicative_update;
            state.weight =
                (1.0 - pull_to_prior) * state.weight +
                pull_to_prior * state.initial_weight;
        }
        else
        {
            state.weight =
                0.95 * state.weight +
                0.05 * state.initial_weight;
        }

        state.weight = std::max(state.initial_weight * min_weight_ratio, state.weight);
        state.weight = std::min(state.initial_weight * max_weight_ratio, state.weight);

        total_weight += state.weight;

        state.segment_score = 0.0;
        state.segment_improvement = 0.0;
        state.segment_uses = 0;
        state.segment_successes = 0;
        state.segment_accepts = 0;
        state.segment_improvements = 0;
        state.segment_new_bests = 0;
    }

    const double rescale_factor =
        total_weight > 0.0 && target_total_weight > 0.0
            ? target_total_weight / total_weight
            : 1.0;

    for (int idx = 0; idx < (int)(operator_state.size()); ++idx)
    {
        GAMOperatorState &state = operator_state[(size_t)idx];
        state.weight *= rescale_factor;
        selection_weights[(size_t)idx] = state.weight;

        statistics.segment_stats.push_back(GAMSegmentStatistics{
            segment_index,
            iteration,
            idx,
            state.weight,
        });
    }
}

std::vector<int> all_operator_indices(int count)
{
    std::vector<int> indices(std::max(0, count));
    std::iota(indices.begin(), indices.end(), 0);
    return indices;
}

std::vector<int> resolve_operator_pool(
    const std::vector<NamedOperator> &ops,
    const std::vector<std::string> &requested_names)
{
    if (ops.empty())
    {
        return {};
    }

    std::vector<int> resolved;
    std::unordered_set<int> seen_indices;
    for (const std::string &name : requested_names)
    {
        for (int idx = 0; idx < (int)(ops.size()); ++idx)
        {
            if (ops[(size_t)idx].name == name && seen_indices.insert(idx).second)
            {
                resolved.push_back(idx);
                break;
            }
        }
    }

    if (resolved.empty())
    {
        return all_operator_indices((int)ops.size());
    }

    return resolved;
}

std::vector<double> build_active_selection_weights(
    const std::vector<double> &selection_weights,
    const std::vector<int> &active_indices)
{
    std::vector<double> active_weights;
    active_weights.reserve(active_indices.size());
    for (int idx : active_indices)
    {
        active_weights.push_back(selection_weights[(size_t)idx]);
    }

    return active_weights;
}

std::vector<NamedOperator> build_active_ops(
    const std::vector<NamedOperator> &ops,
    const std::vector<int> &active_indices)
{
    std::vector<NamedOperator> active_ops;
    active_ops.reserve(active_indices.size());
    for (int idx : active_indices)
    {
        active_ops.push_back(ops[(size_t)idx]);
    }

    return active_ops;
}

bool segment_has_activity(const std::vector<GAMOperatorState> &operator_state)
{
    for (const GAMOperatorState &state : operator_state)
    {
        if (state.segment_uses > 0)
        {
            return true;
        }
    }

    return false;
}
} // namespace

GAMResult general_adaptive_metaheuristic(
    const Instance &instance,
    Solution initial,
    const std::vector<NamedOperator> &ops,
    int time_limit_s,
    const GAMConfig &config,
    const std::vector<double> &initial_weights)
{
    // TEST: With NN initial solution (easier to start with than greedy drone cover solution)
    initial = nearest_neighbour(instance);
    // initial = greedy_drone_cover(instance, initial);

    const bool timed_mode = time_limit_s > 0;
    constexpr int max_iterations = 10000;
    const int segment_length = 100;
    const int stopping_condition = 400;
    const double phase_one_fraction =
        std::clamp(config.phase_one_fraction, 0.0, 1.0);
    const double phase_one_until_s = std::max(0.0, time_limit_s * phase_one_fraction);

    GAMResult result;
    result.statistics.max_iterations = timed_mode ? 0 : max_iterations;
    result.statistics.segment_length = timed_mode ? 0 : segment_length;
    result.statistics.stopping_condition = stopping_condition;
    result.statistics.best_found_iteration = 0;
    if (!timed_mode)
    {
        result.statistics.iteration_stats.reserve(max_iterations);
        result.statistics.segment_stats.reserve(
            ((max_iterations + segment_length - 1) / segment_length) * ops.size());
    }

    if (ops.empty())
    {
        result.solution = std::move(initial);
        return result;
    }

    const std::vector<int> phase_one_indices =
        resolve_operator_pool(ops, config.phase_one_operator_names);
    const std::vector<int> phase_two_indices =
        resolve_operator_pool(ops, config.phase_two_operator_names);
    const std::vector<NamedOperator> phase_one_ops =
        build_active_ops(ops, phase_one_indices);
    const std::vector<NamedOperator> phase_two_ops =
        build_active_ops(ops, phase_two_indices);

    const std::vector<double> initial_selection_weights = initialize_weights(ops, initial_weights);
    std::vector<GAMOperatorState> operator_state =
        build_operator_state(ops, initial_selection_weights);
    std::vector<double> selection_weights = build_selection_weights(operator_state);

    GAMSolutionCache solution_cache;
    const long long initial_cost = objective_function_impl(instance, initial);
    gam_cache_known_feasible_solution(solution_cache, instance, initial, initial_cost);
    const double sampled_positive_delta = average_positive_delta_sample(
        instance,
        initial,
        initial_cost,
        phase_one_ops.empty() ? ops : phase_one_ops,
        &solution_cache);

    const double target_initial_worsening_acceptance =
        instance.n >= 50 ? 0.12 : 0.18;
    const double initial_temperature =
        -sampled_positive_delta / std::log(target_initial_worsening_acceptance);
    double phase_initial_temperature = initial_temperature;
    double phase_final_temperature = std::max(1.0, phase_initial_temperature * 0.05);
    double timed_cooling_ratio =
        phase_final_temperature / phase_initial_temperature;
    double cooling_rate = std::pow(
        phase_final_temperature / phase_initial_temperature,
        1.0 / std::max(1, max_iterations - 1));

    double temperature = phase_initial_temperature;
    double reward_delta_scale = std::max(1.0, sampled_positive_delta);
    std::uniform_real_distribution<double> unit_dist(0.0, 1.0);

    result.statistics.operator_names.reserve(operator_state.size());
    result.statistics.operator_stats.reserve(operator_state.size());
    for (const GAMOperatorState &state : operator_state)
    {
        result.statistics.operator_names.push_back(state.name);
        result.statistics.operator_stats.push_back(GAMOperatorStatistics{
            (int)(result.statistics.operator_stats.size()),
            state.name});
    }

    Solution incumbent = std::move(initial);
    long long incumbent_cost = initial_cost;
    Solution best = incumbent;
    long long best_cost = incumbent_cost;
    int non_improving_iterations = 0;
    int iteration = 0;

    const auto run_start = std::chrono::steady_clock::now();
    double phase_schedule_start_elapsed_s = 0.0;
    double last_phase_schedule_elapsed_s = 0.0;
    double phase_schedule_duration_s =
        timed_mode ? std::max(0.001, phase_one_until_s) : 0.0;
    const double segment_duration_s =
        timed_mode ? std::max(0.001, (double)time_limit_s / 50.0) : 0.0;
    int completed_time_segments = 0;
    bool previous_phase_one = true;

    while (true)
    {
        const double elapsed_before_iteration = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - run_start)
                                                   .count();
        if (timed_mode)
        {
            if (iteration > 0 && elapsed_before_iteration >= (double)time_limit_s)
            {
                break;
            }
        }
        else if (iteration >= max_iterations)
        {
            break;
        }

        ++iteration;
        const bool phase_one =
            !timed_mode || elapsed_before_iteration < phase_one_until_s;
        const std::vector<int> &active_indices =
            phase_one ? phase_one_indices : phase_two_indices;
        const std::vector<NamedOperator> &active_ops =
            phase_one ? phase_one_ops : phase_two_ops;

        if (timed_mode &&
            config.reset_acceptance_each_phase &&
            previous_phase_one &&
            !phase_one)
        {
            phase_schedule_start_elapsed_s = elapsed_before_iteration;
            last_phase_schedule_elapsed_s = 0.0;
            phase_schedule_duration_s = std::max(
                0.001,
                (double)time_limit_s - phase_schedule_start_elapsed_s);
            if (config.acceptance_mode == GAMAcceptanceMode::SimulatedAnnealing)
            {
                phase_initial_temperature = build_phase_initial_temperature(
                    instance,
                    incumbent,
                    incumbent_cost,
                    active_ops.empty() ? ops : active_ops,
                    &solution_cache);
                phase_final_temperature = std::max(1.0, phase_initial_temperature * 0.05);
                timed_cooling_ratio = phase_final_temperature / phase_initial_temperature;
                temperature = phase_initial_temperature;
            }
            non_improving_iterations = 0;
        }
        previous_phase_one = phase_one;

        if (non_improving_iterations >= stopping_condition)
        {
            const int remaining_iterations = std::max(0, max_iterations - iteration + 1);
            const int escape_budget = 20;
            const std::vector<double> active_weights =
                build_active_selection_weights(selection_weights, active_indices);
            const GAMEscapeResult escape_result = run_configured_escape(
                instance,
                incumbent,
                active_ops.empty() ? ops : active_ops,
                active_weights.empty() ? selection_weights : active_weights,
                escape_budget,
                config.escape_mode,
                &solution_cache);
            incumbent = escape_result.incumbent;
            incumbent_cost = escape_result.incumbent_cost;

            if (escape_result.found_new_best)
            {
                if (escape_result.best_seen_cost < best_cost)
                {
                    best = escape_result.best_seen;
                    best_cost = escape_result.best_seen_cost;
                    result.statistics.best_updates++;
                    result.statistics.best_found_iteration = iteration;
                }
            }
            else if (incumbent_cost < best_cost)
            {
                best = incumbent;
                best_cost = incumbent_cost;
                result.statistics.best_updates++;
                result.statistics.best_found_iteration = iteration;
            }

            if (config.acceptance_mode == GAMAcceptanceMode::SimulatedAnnealing)
            {
                temperature = std::max(temperature, phase_initial_temperature * 0.25);
            }
            // solution_cache.clear();
            non_improving_iterations = 0;
        }

        const std::vector<double> active_selection_weights =
            build_active_selection_weights(selection_weights, active_indices);
        const int active_selected_idx = roulette_wheel_selection(
            active_selection_weights.empty()
                ? selection_weights
                : active_selection_weights);
        const int selected_idx =
            active_selection_weights.empty()
                ? active_selected_idx
                : active_indices[(size_t)active_selected_idx];
        GAMOperatorState &selected = operator_state[(size_t)selected_idx];
        selected.segment_uses++;

        const double schedule_elapsed =
            timed_mode
                ? std::max(0.0, elapsed_before_iteration - phase_schedule_start_elapsed_s)
                : (double)std::max(0, iteration - 1);
        const double schedule_duration =
            timed_mode
                ? std::max(0.001, phase_schedule_duration_s)
                : (double)std::max(1, max_iterations - 1);
        const double schedule_progress = std::clamp(
            schedule_elapsed / schedule_duration,
            0.0,
            1.0);
        const double allowed_deviation =
            config.acceptance_mode == GAMAcceptanceMode::BestRelativeRRT
                ? compute_allowed_deviation(
                      best_cost,
                      schedule_elapsed,
                      schedule_duration,
                      config.allowed_deviation_fraction)
                : 0.0;

        GAMIterationStatistics iteration_stat;
        iteration_stat.iteration = iteration;
        iteration_stat.operator_idx = selected_idx;
        iteration_stat.temperature =
            config.acceptance_mode == GAMAcceptanceMode::BestRelativeRRT
                ? allowed_deviation
                : temperature;
        iteration_stat.allowed_deviation = allowed_deviation;
        iteration_stat.virtual_schedule_fraction = schedule_progress;
        GAMOperatorStatistics &operator_statistics =
            result.statistics.operator_stats[(size_t)selected_idx];
        operator_statistics.uses++;

        const auto evaluation_start = std::chrono::steady_clock::now();
        Solution neighbour = incumbent;
        if (!ops[(size_t)selected_idx].op(instance, neighbour))
        {
            result.statistics.operator_failures++;
            operator_statistics.failures++;
            non_improving_iterations++;
        }
        else if (same_solution(neighbour, incumbent))
        {
            result.statistics.operator_failures++;
            operator_statistics.failures++;
            non_improving_iterations++;
        }
        else
        {
            const GAMSolutionEvaluation evaluation =
                evaluate_solution_with_cache(instance, neighbour, &solution_cache);

            if (!evaluation.feasible || !evaluation.objective_known)
            {
                result.statistics.infeasible_candidates++;
                operator_statistics.changed_candidates++;
                operator_statistics.infeasible_candidates++;
                non_improving_iterations++;
            }
            else
            {
                selected.segment_successes++;
                operator_statistics.changed_candidates++;
                operator_statistics.feasible_candidates++;

                const long long cost = evaluation.objective;
                const long long delta_e = cost - incumbent_cost;

                iteration_stat.delta = delta_e;
                iteration_stat.has_delta = true;
                operator_statistics.delta_sum += delta_e;
                operator_statistics.delta_samples++;

                bool accept = delta_e <= 0;
                double acceptance_probability = 1.0;

                if (delta_e > 0)
                {
                    if (config.acceptance_mode == GAMAcceptanceMode::BestRelativeRRT)
                    {
                        accept =
                            static_cast<double>(cost) <=
                            static_cast<double>(best_cost) + allowed_deviation;
                    }
                    else
                    {
                        acceptance_probability =
                            compute_acceptance_probability(delta_e, temperature);
                        accept = unit_dist(gen) < acceptance_probability;
                        iteration_stat.worsening_acceptance_probability = acceptance_probability;
                    }
                }

                if (accept)
                {
                    selected.segment_accepts++;
                    operator_statistics.accepted_moves++;
                    incumbent = neighbour;
                    incumbent_cost = cost;
                    result.statistics.accepted_moves++;

                    if (delta_e < 0)
                    {
                        result.statistics.improving_accepts++;
                        operator_statistics.improving_accepts++;
                    }
                    else
                    {
                        result.statistics.non_improving_accepts++;
                    }
                }

                const bool new_best = cost < best_cost;
                const bool incumbent_improved = accept && delta_e < 0;

                if (delta_e < 0)
                {
                    selected.segment_improvements++;
                    selected.segment_improvement +=
                        std::min(5.0, (double)(-delta_e) / std::max(1.0, reward_delta_scale));
                }

                if (new_best)
                {
                    selected.segment_new_bests++;
                    operator_statistics.new_bests++;
                    best = neighbour;
                    best_cost = cost;
                    result.statistics.best_updates++;
                    result.statistics.best_found_iteration = iteration;
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

                const double reward =
                    compute_operator_reward(
                        delta_e,
                        accept,
                        new_best,
                        evaluation.is_new_solution,
                        reward_delta_scale);
                selected.segment_score += reward;

                if (delta_e != 0)
                {
                    const double abs_delta = std::abs((double)delta_e);
                    reward_delta_scale = 0.95 * reward_delta_scale + 0.05 * std::max(1.0, abs_delta);
                }
            }
        }

        iteration_stat.runtime_ms = std::max(
            0.0,
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - evaluation_start)
                .count());
        iteration_stat.incumbent_objective = incumbent_cost;
        iteration_stat.best_objective = best_cost;
        operator_statistics.total_runtime_ms += iteration_stat.runtime_ms;
        result.statistics.iteration_stats.push_back(iteration_stat);

        if (timed_mode)
        {
            const double elapsed_after_iteration = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - run_start)
                                                        .count();
            const double phase_elapsed_after_iteration = std::max(
                0.0,
                elapsed_after_iteration - phase_schedule_start_elapsed_s);
            const double elapsed_delta_s = std::max(
                0.0,
                phase_elapsed_after_iteration - last_phase_schedule_elapsed_s);
            const double progress_delta =
                elapsed_delta_s / std::max(0.001, phase_schedule_duration_s);
            if (config.acceptance_mode == GAMAcceptanceMode::SimulatedAnnealing &&
                progress_delta > 0.0)
            {
                temperature = std::max(
                    phase_final_temperature,
                    temperature * std::pow(timed_cooling_ratio, progress_delta));
            }
            last_phase_schedule_elapsed_s = phase_elapsed_after_iteration;

            if (segment_duration_s > 0.0)
            {
                const int target_completed_segments = std::min(
                    50,
                    (int)std::floor(elapsed_after_iteration / segment_duration_s));
                if (target_completed_segments > completed_time_segments &&
                    segment_has_activity(operator_state))
                {
                    update_operator_weights(
                        operator_state,
                        selection_weights,
                        target_completed_segments,
                        iteration,
                        result.statistics);
                    completed_time_segments = target_completed_segments;
                }
            }
        }
        else
        {
            if (iteration % segment_length == 0)
            {
                update_operator_weights(
                    operator_state,
                    selection_weights,
                    iteration / segment_length,
                    iteration,
                    result.statistics);
            }

            if (config.acceptance_mode == GAMAcceptanceMode::SimulatedAnnealing)
            {
                temperature = std::max(phase_final_temperature, temperature * cooling_rate);
            }
        }
    }

    if (timed_mode)
    {
        if (segment_has_activity(operator_state))
        {
            update_operator_weights(
                operator_state,
                selection_weights,
                completed_time_segments + 1,
                iteration,
                result.statistics);
        }
    }
    else if (max_iterations % segment_length != 0)
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

GAMResult general_adaptive_metaheuristic(
    const Instance &instance,
    Solution initial,
    const std::vector<NamedOperator> &ops,
    const std::vector<double> &initial_weights)
{
    return general_adaptive_metaheuristic(
        instance,
        std::move(initial),
        ops,
        0,
        GAMConfig{},
        initial_weights);
}
