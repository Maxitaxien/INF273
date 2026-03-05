#pragma once
#include <map>
#include <string>
#include "runners/wrappers.h"

namespace algorithms
{
    inline const std::map<std::string, Algorithm> all = {
        {"Random Search", blind_random_wrapper},
        {"Local Search 1-insert", local_search_wrapper},
        {"Simulated Annealing 1-insert", sa_wrapper},
    };
}