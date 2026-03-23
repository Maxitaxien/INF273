#include "datahandling/save_to_csv.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

std::string BASE = fs::current_path().parent_path().string() + "/runs/";

namespace
{
std::string clean_dataset(const std::string& dataset)
{
    size_t last_slash = dataset.find_last_of("/\\");
    size_t last_dot = dataset.find_last_of(".");

    size_t start = (last_slash == std::string::npos) ? 0 : last_slash + 1;
    size_t length = (last_dot == std::string::npos || last_dot < start)
        ? std::string::npos
        : last_dot - start;

    return dataset.substr(start, length);
}

std::string gam_run_stem(const std::string& run_dir, const std::string& dataset, int run_idx)
{
    return run_dir + "/" + clean_dataset(dataset) + "_gam_run_" + std::to_string(run_idx);
}

std::string operator_name_at(const GAMRunStatistics& statistics, int operator_idx)
{
    if (operator_idx < 0 || operator_idx >= (int)(statistics.operator_names.size()))
    {
        return "";
    }

    return statistics.operator_names[operator_idx];
}
}

bool save_to_csv(
    const std::string& run_dir,
    const std::string& algo_name,
    const std::string& dataset,
    long double avg,
    long long best,
    double improvement_percent,
    long double avg_runtime,
    const std::string& solution_str
)
{
    std::string csv_path = run_dir + "/" + clean_dataset(dataset) + ".csv";
    std::ofstream csvfile(csv_path);

    std::string solution_path = run_dir + "/" + clean_dataset(dataset) + ".txt";
    std::ofstream solutionfile(solution_path);

    if (!csvfile.is_open())
    {
        std::cerr << "Failed to open file at:" << csv_path << "\n";
        return false;
    }

    csvfile << clean_dataset(dataset) << ",,,,\n";
    csvfile << " ,Average objective,Best Objective,Improvement (%),Average running time (in seconds)\n";
    csvfile << algo_name << "," << avg << "," << best << "," << improvement_percent << "," << avg_runtime << "\n";

    if (!solutionfile.is_open())
    {
        std::cerr << "Failed to open file at:" << solution_path << "\n";
        return false;
    }

    solutionfile << solution_str << "\n";

    return true;
}

bool save_gam_statistics(
    const std::string& run_dir,
    const std::string& dataset,
    int run_idx,
    const GAMRunStatistics& statistics)
{
    const std::string stem = gam_run_stem(run_dir, dataset, run_idx);

    std::ofstream summary_file(stem + "_summary.csv");
    if (!summary_file.is_open())
    {
        std::cerr << "Failed to open file at:" << stem << "_summary.csv\n";
        return false;
    }

    summary_file
        << "run,max_iterations,segment_length,stopping_condition,best_found_iteration,"
        << "operator_failures,infeasible_candidates,accepted_moves,improving_accepts,"
        << "non_improving_accepts,best_updates\n";
    summary_file
        << run_idx << ","
        << statistics.max_iterations << ","
        << statistics.segment_length << ","
        << statistics.stopping_condition << ","
        << statistics.best_found_iteration << ","
        << statistics.operator_failures << ","
        << statistics.infeasible_candidates << ","
        << statistics.accepted_moves << ","
        << statistics.improving_accepts << ","
        << statistics.non_improving_accepts << ","
        << statistics.best_updates << "\n";

    std::ofstream iteration_file(stem + "_iterations.csv");
    if (!iteration_file.is_open())
    {
        std::cerr << "Failed to open file at:" << stem << "_iterations.csv\n";
        return false;
    }

    iteration_file
        << "iteration,operator_idx,operator_name,selected_weight,incumbent_cost_before,"
        << "candidate_cost,delta,acceptance_probability,temperature,operator_succeeded,"
        << "candidate_feasible,accepted,improving,new_best,incumbent_cost_after,best_cost_after\n";
    for (const GAMIterationStatistics& row : statistics.iteration_stats)
    {
        iteration_file
            << row.iteration << ","
            << row.operator_idx << ","
            << operator_name_at(statistics, row.operator_idx) << ","
            << row.selected_weight << ","
            << row.incumbent_cost_before << ","
            << row.candidate_cost << ","
            << row.delta << ","
            << row.acceptance_probability << ","
            << row.temperature << ","
            << row.operator_succeeded << ","
            << row.candidate_feasible << ","
            << row.accepted << ","
            << row.improving << ","
            << row.new_best << ","
            << row.incumbent_cost_after << ","
            << row.best_cost_after << "\n";
    }

    std::ofstream segment_file(stem + "_segments.csv");
    if (!segment_file.is_open())
    {
        std::cerr << "Failed to open file at:" << stem << "_segments.csv\n";
        return false;
    }

    segment_file << "segment,iteration,operator_idx,operator_name,weight,segment_score,segment_uses\n";
    for (const GAMSegmentStatistics& row : statistics.segment_stats)
    {
        segment_file
            << row.segment << ","
            << row.iteration << ","
            << row.operator_idx << ","
            << operator_name_at(statistics, row.operator_idx) << ","
            << row.weight << ","
            << row.segment_score << ","
            << row.segment_uses << "\n";
    }

    std::ofstream operator_file(stem + "_operators.csv");
    if (!operator_file.is_open())
    {
        std::cerr << "Failed to open file at:" << stem << "_operators.csv\n";
        return false;
    }

    operator_file
        << "operator_idx,operator_name,final_weight,total_score,total_uses,accepted,"
        << "improving_accepts,best_updates,failures,infeasible\n";
    for (int idx = 0; idx < (int)(statistics.operator_summaries.size()); ++idx)
    {
        const GAMOperatorSummary& row = statistics.operator_summaries[idx];
        operator_file
            << idx << ","
            << row.name << ","
            << row.final_weight << ","
            << row.total_score << ","
            << row.total_uses << ","
            << row.accepted << ","
            << row.improving_accepts << ","
            << row.best_updates << ","
            << row.failures << ","
            << row.infeasible << "\n";
    }

    return true;
}

