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
    const Instance &instance,
    Solution &incumbent,
    long long &incumbent_cost,
    Solution &best,
    long long &best_cost,
    Operator op,
    int warmup_amount);

/**
 *
 * Incunbent: Current solution
 */
Solution simulated_annealing(
    const Instance &instance,
    Solution initial,
    Operator op,
    double TF);