#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"
#include <functional>
#include <vector>

using Operator = std::function<std::vector<Solution>(const Instance&, const Solution&)>;

std::vector<Solution> one_reinsert_operator(const Instance& instance, const Solution& sol);