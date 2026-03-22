#pragma once

#include <vector>

int roulette_wheel_selection_uniform(int n);

int roulette_wheel_selection(const std::vector<double>& fitness);

int roulette_wheel_selection_exponential(int n, double decay = 0.28);
