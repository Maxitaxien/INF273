#pragma once
#include <functional>
#include <string>
#include "datahandling/instance.h"
#include "verification/solution.h"
#include "runners/wrappers.h"



/**
 * Runs a generalized Algorithm.
 */
void run_algorithm(
    const std::string& algo_name,
    Algorithm algo,
    std::function<bool(const Instance&, Solution&)> op,
    std::function<long long(const Instance&, const Solution&)> objective
);