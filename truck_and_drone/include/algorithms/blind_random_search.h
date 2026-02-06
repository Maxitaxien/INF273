#pragma once
#include "verification/solution.h"
#include "datahandling/instance.h"
#include "verification/objective_value.h"
#include <functional>

Solution blind_random_search(
    const Instance& problem_instance, 
    const Solution& initial, 
    std::function<long long(const Instance&, const Solution&)> objective_value_check);