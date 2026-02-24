#pragma once
#include <string>

bool save_to_csv(
    const std::string& run_dir,
    const std::string& algo_name,
    const std::string& dataset,
    long double avg,
    long long best,
    double improvement_percent,
    long double avg_runtime,
    const std::string& solution_str
);