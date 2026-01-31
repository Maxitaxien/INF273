#pragma once
#include <vector>

struct Instance {
    int n;
    int lim;
    std::vector<std::vector<long long>> truck_matrix;
    std::vector<std::vector<long long>> drone_matrix;
};