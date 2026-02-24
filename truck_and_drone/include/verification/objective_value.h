#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Calculates the sum of waiting times in minutes, which is the competition metric.
 */
long long calculate_total_waiting_time(const Instance& problem_instance, const Solution& solution);

long long objective_function_impl(const Instance& instance, const Solution& solution);

/**
 * LLM from scratch computation. Let's see if it works.
 */
long long compute_total_wait_time(const Instance& instance, const Solution& solution);