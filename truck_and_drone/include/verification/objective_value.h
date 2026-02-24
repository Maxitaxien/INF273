#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Calculates the sum of waiting times in minutes, which is the competition metric.
 * LEGACY: This was implemented slightly incorrectly
 */
long long calculate_total_waiting_time(const Instance& problem_instance, const Solution& solution);

/**
 * Preferred objective function.
 */
long long objective_function_impl(const Instance& instance, const Solution& solution);
