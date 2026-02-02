#pragma once

#include <string>
#include <vector>
#include "datahandling/instance.h"
#include "verification/solution.h"

std::vector<std::string> split(const std::string& s, char delimiter);

bool includes_all_nodes(int n, const std::string& submission);

bool all_drone_flights_under_lim(const Instance& problem_instance, const Solution& solution);