#pragma once
#include <map>
#include <string>
#include "runners/wrappers.h"

namespace algorithms
{
    inline const std::map<std::string, Algorithm> all = {
        {"Random Search", blind_random_wrapper},
        {"Local Search", local_search_wrapper},
        {"Simulated Annealing", sa_wrapper},
    };

    inline const std::map<std::string, Algorithm> construction = {
        {"Nearest Neighbour", nearest_neighbour_wrapper},
        {"Construction", construction_wrapper}};

    inline const std::vector<Algorithm> escape = {
        random_escape_wrapper};
}