#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Calculates the sum of waiting times in minutes, which is the competition metric.
 */
long long calculateTotalWaitingTime(const Instance& problem_instance, const Solution& solution);



/**
 * Slightly simplified objective value which is equivalent to the total wait time,
 * but will generate smaller values and can be useful for intermediate checks.
 */
long long getFinalArrivalTime(const Instance& problem_instance, const Solution& solution);