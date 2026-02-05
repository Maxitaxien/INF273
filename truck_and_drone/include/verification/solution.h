#pragma once
#include<vector>
#include<unordered_map>


struct Solution {
    // @brief A vector of ints representing the current truck route
    std::vector<int> truck_route;
    // @brief A map of vectors with keys: n and length of values in each: m, featuring entries like: {0: [(-1, -1), (-1, -1)]} to hold drone info.
    std::unordered_map<int, std::vector<std::pair<int,int>>> drone_map;
};