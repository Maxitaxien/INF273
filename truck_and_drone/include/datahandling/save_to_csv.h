#pragma once
#include "algorithms/gam.h"
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

bool save_gam_statistics(
    const std::string& run_dir,
    const std::string& dataset,
    int run_idx,
    const GAMRunStatistics& statistics,
    const std::vector<GAMRunReport>& run_reports
);
