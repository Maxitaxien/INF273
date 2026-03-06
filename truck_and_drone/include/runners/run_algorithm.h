#pragma once
#include <functional>
#include <string>
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
    std::function<bool(const Instance &, Solution &)> op,
    const std::string &base_dir);

/**
 * Runs all algorithms from algorithms namespace.
 */
void run_all_algos(Operator op);

/**
 * Runs construction algorithms.
 */
void run_construction_algos();