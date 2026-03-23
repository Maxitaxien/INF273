#include <iostream>
#include <chrono>
#include <map>
#include <vector>
#include "runners/run_algorithm.h"
#include "algorithms/gam.h"
#include "datahandling/reader.h"
#include "datahandling/convert_to_submission.h"
#include "datahandling/save_to_csv.h"
#include "datahandling/datasets.h"
#include "algorithms/simple_initial_solution.h"
#include "algorithms/blind_random_search.h"
#include "algorithms/local_search.h"
#include "algorithms/simulated_annealing.h"
#include "operators/operator.h"
#include "verification/objective_value.h"
#include "runners/wrappers.h"
#include "runners/algorithms.h"
#include "datahandling/file_helpers.h"
#include "datahandling/create_markdown_tables.h"
#include "general/roulette_wheel_selection.h"

const long long INF = 4e18;

using namespace datasets;
using namespace algorithms;

Operator make_uniform_weighted_selector(const std::vector<NamedOperator> &ops);
Operator make_tuned_weighted_selector(
    const std::vector<NamedOperator> &ops,
    const std::vector<double> &weights);

namespace
{
std::vector<std::string> benchmark_datasets()
{
    return {f10, f20, f50, f100, r10, r20, r50, r100};
}

std::string build_operator_mix_name(
    const std::vector<NamedOperator> &ops,
    const std::vector<double> &weights,
    const std::string &equal_suffix,
    const std::string &tuned_suffix)
{
    if (ops.empty())
    {
        return "";
    }

    if (ops.size() == 1)
    {
        return ops[0].name;
    }

    return weights.empty() ? equal_suffix : tuned_suffix;
}

NamedOperator build_run_operator(
    const std::vector<NamedOperator> &ops,
    const std::vector<double> &weights,
    const std::string &equal_suffix,
    const std::string &tuned_suffix)
{
    if (ops.empty())
    {
        return make_no_op_operator();
    }

    if (ops.size() == 1)
    {
        return ops[0];
    }

    Operator selector = weights.empty()
        ? make_uniform_weighted_selector(ops)
        : make_tuned_weighted_selector(ops, weights);

    return NamedOperator{
        build_operator_mix_name(ops, weights, equal_suffix, tuned_suffix),
        selector};
}
}

Operator make_uniform_weighted_selector(const std::vector<NamedOperator> &ops)
{
    return [ops](const Instance &instance, Solution &sol) {
        int idx = roulette_wheel_selection_uniform(static_cast<int>(ops.size()));
        return ops[idx].op(instance, sol);
    };
}

Operator make_tuned_weighted_selector(
    const std::vector<NamedOperator> &ops,
    const std::vector<double> &weights)
{
    std::vector<double> fitness = weights;
    if (fitness.size() != ops.size())
    {
        fitness.assign(ops.size(), 1.0);
    }

    return [ops, fitness](const Instance &instance, Solution &sol) {
        int idx = roulette_wheel_selection(fitness);
        return ops[idx].op(instance, sol);
    };
}

void run_algorithm(
    const std::string &algo_name,
    Algorithm algo,
    const NamedOperator &op,
    const std::string &base_dir,
    int amnt_iter)
{
    const long long INF = 4e18;
    const std::vector<std::string> datasets = benchmark_datasets();

    std::string algo_op_name = algo_name;
    if (!op.name.empty())
    {
        algo_op_name += " " + op.name;
    }

    std::string run_dir = create_algo_directory(base_dir, algo_op_name);

    for (const auto &dataset : datasets)
    {
        long long best = INF;
        long double avg = 0;
        double avg_runtime = 0;
        long long initial_obj = 0;
        Instance instance;
        Solution initial;
        Solution best_solution;

        for (int i = 0; i < amnt_iter; i++)
        {
            auto start = std::chrono::high_resolution_clock::now();

            instance = read_instance(dataset);
            initial = simple_initial_solution(instance.n);

            if (i == 0)
            {
                initial_obj = objective_function_impl(instance, initial);
            }

            Solution sol = algo(instance, initial, op.op);
            auto stop = std::chrono::high_resolution_clock::now();

            long long val = objective_function_impl(instance, sol);
            avg += val;
            avg_runtime += std::chrono::duration<double>(stop - start).count();

            if (val < best)
            {
                best = val;
                best_solution = sol;
            }
        }

        avg /= amnt_iter;
        avg_runtime /= amnt_iter;

        double improvement = 100.0 * (initial_obj - best) / initial_obj;

        save_to_csv(run_dir, algo_op_name, dataset, avg, best, improvement, avg_runtime,
                    convert_to_submission(best_solution));
    }
}

void run_all_algos(const NamedOperator &op)
{
    run_all_algos(std::vector<NamedOperator>{op}, {});
}

NamedOperator make_no_op_operator()
{
    return NamedOperator{"", [](const Instance &, Solution &) {
        return false;
    }};
}

void run_all_algos()
{
    run_all_algos(make_no_op_operator());
}

void run_all_algos(const std::vector<NamedOperator> &ops, const std::vector<double> &weights)
{
    int amnt_iter = 10;
    std::string base_dir = create_run_directory();

    NamedOperator run_op = build_run_operator(
        ops,
        weights,
        "New Operators (Equal Weights)",
        "New Operators (Tuned Weights)");

    for (const auto &[name, wrapper] : algorithms::all)
    {
        run_algorithm(name, wrapper, run_op, base_dir, amnt_iter);
    }
    create_markdown_tables(base_dir);
}

void run_gam()
{
    run_gam(make_no_op_operator());
}

void run_gam(const NamedOperator &op)
{
    run_gam(std::vector<NamedOperator>{op}, {});
}

void run_gam(const std::vector<NamedOperator> &ops, const std::vector<double> &weights)
{
    const int amnt_iter = 10;
    const std::vector<std::string> datasets = benchmark_datasets();
    std::string base_dir = create_run_directory();

    std::string algo_op_name = "General Adaptive Metaheuristic";
    const std::string mix_name = build_operator_mix_name(
        ops,
        weights,
        "Operator Mix (Equal Weights)",
        "Operator Mix (Tuned Weights)");
    if (!mix_name.empty())
    {
        algo_op_name += " " + mix_name;
    }

    std::string run_dir = create_algo_directory(base_dir, algo_op_name);

    for (const auto &dataset : datasets)
    {
        long long best = INF;
        long double avg = 0;
        double avg_runtime = 0;
        long long initial_obj = 0;
        Instance instance;
        Solution initial;
        Solution best_solution;

        for (int i = 0; i < amnt_iter; ++i)
        {
            auto start = std::chrono::high_resolution_clock::now();

            instance = read_instance(dataset);
            initial = simple_initial_solution(instance.n);

            if (i == 0)
            {
                initial_obj = objective_function_impl(instance, initial);
            }

            Solution sol = general_adaptive_metaheuristic(instance, initial, ops, weights);
            auto stop = std::chrono::high_resolution_clock::now();

            const long long val = objective_function_impl(instance, sol);
            avg += val;
            avg_runtime += std::chrono::duration<double>(stop - start).count();

            if (val < best)
            {
                best = val;
                best_solution = sol;
            }
        }

        avg /= amnt_iter;
        avg_runtime /= amnt_iter;

        const double improvement = 100.0 * (initial_obj - best) / initial_obj;
        save_to_csv(
            run_dir,
            algo_op_name,
            dataset,
            avg,
            best,
            improvement,
            avg_runtime,
            convert_to_submission(best_solution));
    }

    create_markdown_tables(base_dir);
}

void run_construction_algos()
{
    int amnt_iter = 1; // construction is deterministc
    std::string base_dir = create_run_directory();

    NamedOperator noop = make_no_op_operator(); // dummy operator - will not be used
    for (const auto &[name, wrapper] : algorithms::construction)
    {
        run_algorithm(name, wrapper, noop, base_dir, amnt_iter);
    }
    create_markdown_tables(base_dir);
}
