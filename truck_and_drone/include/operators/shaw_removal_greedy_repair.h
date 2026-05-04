#pragma once

#include "datahandling/instance.h"
#include "verification/solution.h"

bool shaw_removal_greedy_repair(const Instance &inst, Solution &sol, int remove_count);