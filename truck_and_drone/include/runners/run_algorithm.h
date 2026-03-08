#pragma once
#include <functional>
#include <string>
#include <vector>
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
 */
void run_all_algos(const std::vector<NamedOperator> &ops);

/**
 * Runs construction algorithms.
 */
void run_construction_algos();