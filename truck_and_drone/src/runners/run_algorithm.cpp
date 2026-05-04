#include <iostream>
#include <chrono>
#include <cstdlib>
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
#include <filesystem>
#include <unordered_map>

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

int default_gam_time_budget(const Instance &instance)
{
    switch (instance.n)
    {
    case 10:
        return 5;
    case 20:
        return 20;
    case 50:
        return 60;
    case 100:
        return 510;
    default:
        return 0;
    }
}

int resolve_gam_time_budget(
    const Instance &instance,
    const std::string &dataset,
    const std::unordered_map<std::string, int> &time_budget_overrides)
{
    const auto exact_it = time_budget_overrides.find(dataset);
    if (exact_it != time_budget_overrides.end())
    {
        return exact_it->second;
    }

    const auto stem_it = time_budget_overrides.find(dataset_stem(dataset));
    if (stem_it != time_budget_overrides.end())
    {
        return stem_it->second;
    }

    return default_gam_time_budget(instance);
}

std::string shell_quote(const std::string &value)
{
    std::string result = "'";
    for (char ch : value)
    {
        if (ch == '\'')
        {
            result += "'\\''";
        }
        else
        {
            result += ch;
        }
    }
    result += "'";
    return result;
}

std::filesystem::path locate_plot_script()
{
    const std::vector<std::filesystem::path> candidates = {
        std::filesystem::current_path() / "scripts" / "plot_run_statistics.py",
        std::filesystem::current_path().parent_path() / "scripts" / "plot_run_statistics.py",
        std::filesystem::current_path().parent_path().parent_path() / "scripts" / "plot_run_statistics.py",
    };

    for (const std::filesystem::path &candidate : candidates)
    {
        if (std::filesystem::exists(candidate))
        {
            return candidate;
        }
    }

    return {};
}

std::string sanitize_path_component(const std::string &value)
{
    std::string sanitized = value;
    for (char &ch : sanitized)
    {
        if (ch == '/' || ch == '\\')
        {
            ch = '-';
        }
    }
    return sanitized;
}

void generate_run_plots(const std::string &run_dir)
{
    const std::filesystem::path script_path = locate_plot_script();
    if (script_path.empty())
    {
        std::cerr << "Plot script not found. Skipping plot generation.\n";
        return;
    }

    const std::string command =
        "python3 " + shell_quote(script_path.string()) + " " + shell_quote(run_dir);

    const int exit_code = std::system(command.c_str());
    if (exit_code != 0)
    {
        std::cerr << "Plot generation failed for run directory: " << run_dir << "\n";
    }
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

void save_best_solution_visualization(
    const std::string &run_dir,
    const std::string &dataset,
    const Instance &instance,
    const Solution &solution)
{
    const std::filesystem::path output_path =
        std::filesystem::path(create_dataset_statistics_directory(run_dir, dataset)) /
        "best_solution.jpg";

    if (!solution.save_visualization(instance, output_path.string()))
    {
        std::cerr << "Failed to save solution visualization at: "
                  << output_path.string() << "\n";
    }
}

void run_single_gam_experiment(
    const std::string &base_dir,
    const std::string &algo_op_name,
    const std::vector<NamedOperator> &ops,
    const std::vector<double> &weights,
    const GAMConfig &config,
    const std::vector<std::string> &datasets,
    int amnt_iter,
    const std::unordered_map<std::string, int> &time_budget_overrides)
{
    std::string run_dir = create_algo_directory(
        base_dir,
        sanitize_path_component(algo_op_name));

    for (const auto &dataset : datasets)
    {
        long long best = INF;
        long double avg = 0;
        double avg_runtime = 0;
        long long initial_obj = 0;
        int best_run_idx = 1;
        Instance instance = read_instance(dataset);
        Solution initial;
        Solution best_solution;
        GAMRunStatistics best_statistics;
        std::vector<GAMRunReport> run_reports;
        run_reports.reserve(amnt_iter);

        for (int i = 0; i < amnt_iter; ++i)
        {
            auto start = std::chrono::steady_clock::now();

            initial = simple_initial_solution(instance.n);

            if (i == 0)
            {
                initial_obj = objective_function_impl(instance, initial);
            }

            const int time_limit_s = resolve_gam_time_budget(
                instance,
                dataset,
                time_budget_overrides);
            GAMResult gam_result = general_adaptive_metaheuristic(
                instance,
                initial,
                ops,
                time_limit_s,
                config,
                weights);
            auto stop = std::chrono::steady_clock::now();

            const long long val = objective_function_impl(instance, gam_result.solution);
            avg += val;
            avg_runtime += std::chrono::duration<double>(stop - start).count();
            run_reports.push_back(GAMRunReport{i + 1, val, gam_result.statistics.best_found_iteration});

            if (val < best)
            {
                best = val;
                best_solution = std::move(gam_result.solution);
                best_statistics = std::move(gam_result.statistics);
                best_run_idx = i + 1;
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
            convert_to_submission(instance, best_solution));
        save_gam_statistics(run_dir, dataset, best_run_idx, best_statistics, run_reports);
        save_best_solution_visualization(run_dir, dataset, instance, best_solution);
    }
}
}

Operator make_uniform_weighted_selector(const std::vector<NamedOperator> &ops)
{
    return [ops](const Instance &instance, Solution &sol) {
        int idx = roulette_wheel_selection_uniform((int)(ops.size()));
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

        instance = read_instance(dataset);

        for (int i = 0; i < amnt_iter; i++)
        {
            auto start = std::chrono::steady_clock::now();

            initial = simple_initial_solution(instance.n);

            if (i == 0)
            {
                initial_obj = objective_function_impl(instance, initial);
            }

            Solution sol = algo(instance, initial, op.op);
            auto stop = std::chrono::steady_clock::now();

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
                    convert_to_submission(instance, best_solution));
        save_best_solution_visualization(run_dir, dataset, instance, best_solution);
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
    generate_run_plots(base_dir);
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
    run_gam(
        ops,
        weights,
        GAMConfig{},
        benchmark_datasets(),
        10,
        {});
}

void run_gam(
    const std::vector<NamedOperator> &ops,
    const std::vector<double> &weights,
    const GAMConfig &config,
    const std::vector<std::string> &datasets,
    int amnt_iter,
    const std::unordered_map<std::string, int> &time_budget_overrides)
{
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
    run_single_gam_experiment(
        base_dir,
        algo_op_name,
        ops,
        weights,
        config,
        datasets,
        amnt_iter,
        time_budget_overrides);

    create_markdown_tables(base_dir);
    generate_run_plots(base_dir);
}

void run_gam_experiments(
    const std::vector<GAMExperiment> &experiments,
    const std::vector<std::string> &datasets,
    int amnt_iter,
    const std::unordered_map<std::string, int> &time_budget_overrides)
{
    if (experiments.empty())
    {
        return;
    }

    std::string base_dir = create_run_directory();
    for (const GAMExperiment &experiment : experiments)
    {
        std::string algo_op_name = "General Adaptive Metaheuristic";
        if (!experiment.label.empty())
        {
            algo_op_name += " " + experiment.label;
        }

        run_single_gam_experiment(
            base_dir,
            algo_op_name,
            experiment.ops,
            experiment.weights,
            experiment.config,
            datasets,
            amnt_iter,
            time_budget_overrides);
    }

    create_markdown_tables(base_dir);
    generate_run_plots(base_dir);
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
    generate_run_plots(base_dir);
}


