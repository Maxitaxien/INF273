#pragma once
#include <vector>

struct Instance {
    int n;
    int lim;
    std::vector<std::vector<float>> truck_matrix;
    std::vector<std::vector<float>> drone_matrix;
};