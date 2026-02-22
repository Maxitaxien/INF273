#pragma once
#include "verification/solution.h"
#include "operators/operator.h"
#include "datahandling/instance.h"

/**
 * Runs warmup phase for warmup_amount iterations.
 * This gives a delta avg value to use in determining initial temperature
 * 
 * @return delta_avg: Float value representing mean of delta_w values.
 */
double warmup_phase(
    const Instance& instance,
    const Solution& solution,
    long long best_cost,
    Operator op,
    int warmup_amount,
    std::function<long long(const Instance&, const Solution&)> objective
);

/**
 * 
 * Incunbent: Current solution
 */
Solution simulated_annealing(const Instance& instance,
                      const Solution& initial,
                      Operator op,
                      double TF,
                      std::function<long long(const Instance&, const Solution&)> objective);