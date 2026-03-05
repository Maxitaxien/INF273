#include <iostream>
#include <chrono>
#include <map>
#include "runners/run_algorithm.h"
#include "datahandling/reader.h"
#include "datahandling/convert_to_submission.h"
#include "datahandling/save_to_csv.h"
#include "datahandling/datasets.h"
#include "algorithms/simple_initial_solution.h"
#include "algorithms/blind_random_search.h"
#include "algorithms/local_search.h"
#include "algorithms/simulated_annealing.h"
#include "verification/objective_value.h"
#include "runners/wrappers.h"
#include "runners/algorithms.h"
#include "datahandling/file_helpers.h"
#include "datahandling/create_markdown_tables.h"

const long long INF = 4e18;

using namespace datasets;
using namespace algorithms;

void run_algorithm(
    const std::string &algo_name,
    Algorithm algo,
    std::function<bool(const Instance &, Solution &)> op,
    std::function<long long(const Instance &, const Solution &)> objective,
    const std::string &base_dir)
{
    const long long INF = 4e18;
    int amnt_iter = 10;
    std::vector<std::string> datasets = {f10, f20, f50, f100, r10, r20, r50, r100};

    std::string run_dir = create_algo_directory(base_dir, algo_name);

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
                initial_obj = objective(instance, initial);
            }

            Solution sol = algo(instance, initial, op, objective);
            auto stop = std::chrono::high_resolution_clock::now();

            long long val = objective(instance, sol);
            avg += val;
            avg_runtime += std::chrono::duration<double>(stop - start).count();

            // Only update best_solution when val improves best
            if (val < best)
            {
                best = val;
                best_solution = sol;
            }
        }

        avg /= amnt_iter;
        avg_runtime /= amnt_iter;

        double improvement = 100.0 * (initial_obj - best) / initial_obj;

        save_to_csv(run_dir, algo_name, dataset, avg, best, improvement, avg_runtime,
                    convert_to_submission(best_solution));
    }
}

void run_all_algos()
{
    std::string base_dir = create_run_directory();
    for (const auto &[name, wrapper] : algorithms::all)
    {
        run_algorithm(name, wrapper, one_reinsert_random, objective_function_impl, base_dir);
    }
    create_markdown_tables(base_dir);
}