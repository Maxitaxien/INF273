#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"

enum class ExchangeKSizes
{
    S = 3,
    M = 4,
    L = 8,
};

bool exchange_k_small(const Instance &instance, Solution &solution);

bool exchange_k_medium(const Instance &instance, Solution &solution);

bool exchange_k_large(const Instance &instance, Solution &solution);
