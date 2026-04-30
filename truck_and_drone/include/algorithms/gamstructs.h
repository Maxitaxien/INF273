#pragma once
#include <string>
#include <vector>
#include "verification/solution.h"

struct GAMIterationStatistics
{
    int iteration = 0;
    int operator_idx = -1;
    long long delta = 0;
    double temperature = 0.0;
    double worsening_acceptance_probability = -1.0;
    double runtime_ms = 0.0;
    bool has_delta = false;
    long long incumbent_objective = 0;
    long long best_objective = 0;
};

struct GAMSegmentStatistics
{
    int segment = 0;
    int iteration = 0;
    int operator_idx = -1;
    double weight = 0.0;
};

struct GAMOperatorStatistics
{
    int operator_idx = -1;
    std::string operator_name;
    int uses = 0;
    int successful_calls = 0;
    int failures = 0;
    int infeasible_candidates = 0;
    int feasible_candidates = 0;
    int accepted_moves = 0;
    int improving_accepts = 0;
    int new_bests = 0;
    int delta_samples = 0;
    long long delta_sum = 0;
    double total_runtime_ms = 0.0;
};

struct GAMRunStatistics
{
    int max_iterations = 0;
    int segment_length = 0;
    int stopping_condition = 0;
    int best_found_iteration = 0;
    int operator_failures = 0;
    int infeasible_candidates = 0;
    int accepted_moves = 0;
    int improving_accepts = 0;
    int non_improving_accepts = 0;
    int best_updates = 0;
    std::vector<std::string> operator_names;
    std::vector<GAMIterationStatistics> iteration_stats;
    std::vector<GAMSegmentStatistics> segment_stats;
    std::vector<GAMOperatorStatistics> operator_stats;
};

struct GAMRunReport
{
    int run = 0;
    long long final_objective = 0;
    int best_found_iteration = 0;
};

struct GAMResult
{
    Solution solution;
    GAMRunStatistics statistics;
};
