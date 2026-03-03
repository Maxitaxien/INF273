#pragma once

#include "verification/solution.h"
#include "datahandling/instance.h"
#include <vector>

/**
 * Get truck times at specific nodes while taking into consideration waiting times 
 * Used for feasibility checks and for some solution fixer algorithms.
 */
std::vector<long long> get_truck_arrival_times(
    const Instance& instance,
    const Solution& solution,
    std::vector<long long>& drone_available,
    long long& total_drone_arrival     
);