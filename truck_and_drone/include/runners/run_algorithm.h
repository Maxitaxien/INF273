#pragma once
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>
#include "algorithms/gam.h"
#include "datahandling/instance.h"
#include "verification/solution.h"
#include "runners/wrappers.h"
#include "operators/operator.h"

/**
 * Runs a generalized Algorithm.
 *
 * @param base_dir is "" when running directly, creation will be handled automatically
 */
void run_algorithm(
    const std::string &algo_name,
    Algorithm algo,
    const NamedOperator &op,
    const std::string &base_dir,
    int amnt_iter = 10);

/**
 * Runs all algorithms from algorithms namespace with a single operator.
 *
 * This is a convenience overload and will use the operator's name when
 * generating output directories.
 */
NamedOperator make_no_op_operator();

void run_all_algos();

void run_all_algos(const NamedOperator &op);

/**
 * Runs all algorithms from algorithms namespace with multiple operators.
 *
 * If `weights` is empty, weights are uniform (equal).
 */
void run_all_algos(const std::vector<NamedOperator> &ops, const std::vector<double> &weights = {});

/**
 * Runs only the GAM algorithm with a single operator mix.
 */
void run_gam();
void run_gam(const NamedOperator &op);
void run_gam(const std::vector<NamedOperator> &ops, const std::vector<double> &weights = {});

struct GAMExperiment
{
    std::string label;
    std::vector<NamedOperator> ops;
    std::vector<double> weights;
    GAMConfig config;
};

void run_gam(
    const std::vector<NamedOperator> &ops,
    const std::vector<double> &weights,
    const GAMConfig &config,
    const std::vector<std::string> &datasets,
    int amnt_iter,
    const std::unordered_map<std::string, int> &time_budget_overrides = {});
void run_gam_experiments(
    const std::vector<GAMExperiment> &experiments,
    const std::vector<std::string> &datasets,
    int amnt_iter,
    const std::unordered_map<std::string, int> &time_budget_overrides = {});

/**
 * Runs construction algorithms.
 */
void run_construction_algos();
