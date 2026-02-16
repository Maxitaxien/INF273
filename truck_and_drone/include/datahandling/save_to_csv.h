#pragma once
#include <string>

bool save_to_csv(
    std::string algo_name,
    std::string dataset, 
    long double avg, 
    long long best, 
    double improvement_percent, 
    long double avg_runtime,
    std::string solution_str
);