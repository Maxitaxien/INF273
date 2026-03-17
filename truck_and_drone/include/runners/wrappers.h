#pragma once
#include <functional>
#include "datahandling/instance.h"
#include "verification/solution.h"

#include "operators/operator.h"

using Algorithm = std::function<Solution(
    const Instance &,
    Solution,
    Operator)>;

Solution blind_random_wrapper(
    const Instance &instance,
    Solution initial,
    Operator op);

Solution sa_wrapper(
    const Instance &instance,
    Solution initial,
    Operator op);

Solution local_search_wrapper(
    const Instance &instance,
    Solution initial,
    Operator op);

Solution nearest_neighbour_wrapper(
    const Instance &instance,
    Solution initial,
    Operator op);

Solution construction_wrapper(
    const Instance &instance,
    Solution initial,
    Operator op);

Solution random_escape_wrapper(
    const Instance &instance,
    Solution current,
    Operator op);