#include "datahandling/save_to_csv.h"
#include "datahandling/file_helpers.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

namespace
{
fs::path gam_statistics_dir(const std::string& run_dir, const std::string& dataset)
{
    return create_dataset_statistics_directory(run_dir, dataset);
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
    const std::string& dataset,
    long double avg,
    long long best,
    double improvement_percent,
    long double avg_runtime,
    const std::string& solution_str
)
{
    std::string csv_path = run_dir + "/" + dataset_stem(dataset) + ".csv";
    std::ofstream csvfile(csv_path);

    std::string solution_path = run_dir + "/" + dataset_stem(dataset) + ".txt";
    std::ofstream solutionfile(solution_path);

    if (!csvfile.is_open())
    {
        std::cerr << "Failed to open file at:" << csv_path << "\n";
        return false;
    }

    csvfile << dataset_stem(dataset) << ",,,\n";
    csvfile << "Average objective,Best Objective,Improvement (%),Average running time (in seconds)\n";
    csvfile << avg << "," << best << "," << improvement_percent << "," << avg_runtime << "\n";

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
    const GAMRunStatistics& statistics,
    const std::vector<GAMRunReport>& run_reports)
{
    const fs::path statistics_dir = gam_statistics_dir(run_dir, dataset);
    fs::create_directories(statistics_dir);

    std::ofstream summary_file(statistics_dir / "summary.csv");
    if (!summary_file.is_open())
    {
        std::cerr << "Failed to open file at:" << (statistics_dir / "summary.csv").string() << "\n";
        return false;
    }

    summary_file
        << "best_run,best_found_iteration,operator_failures,infeasible_candidates,"
        << "accepted_moves,improving_accepts,non_improving_accepts,best_updates\n";
    summary_file
        << run_idx << ","
        << statistics.best_found_iteration << ","
        << statistics.operator_failures << ","
        << statistics.infeasible_candidates << ","
        << statistics.accepted_moves << ","
        << statistics.improving_accepts << ","
        << statistics.non_improving_accepts << ","
        << statistics.best_updates << "\n";

    std::ofstream iteration_file(statistics_dir / "trace.csv");
    if (!iteration_file.is_open())
    {
        std::cerr << "Failed to open file at:" << (statistics_dir / "trace.csv").string() << "\n";
        return false;
    }

    iteration_file
        << "iteration,operator_idx,operator_name,delta,has_delta,incumbent_objective,"
        << "best_objective,temperature,allowed_deviation,virtual_schedule_fraction,"
        << "worsening_acceptance_probability,runtime_ms\n";
    for (const GAMIterationStatistics& row : statistics.iteration_stats)
    {
        iteration_file
            << row.iteration << ","
            << row.operator_idx << ","
            << operator_name_at(statistics, row.operator_idx) << ","
            << row.delta << ","
            << row.has_delta << ","
            << row.incumbent_objective << ","
            << row.best_objective << ","
            << row.temperature << ","
            << row.allowed_deviation << ","
            << row.virtual_schedule_fraction << ","
            << row.worsening_acceptance_probability << ","
            << row.runtime_ms << "\n";
    }

    std::ofstream weight_file(statistics_dir / "weights.csv");
    if (!weight_file.is_open())
    {
        std::cerr << "Failed to open file at:" << (statistics_dir / "weights.csv").string() << "\n";
        return false;
    }

    weight_file << "segment,iteration,operator_idx,operator_name,weight\n";
    for (const GAMSegmentStatistics& row : statistics.segment_stats)
    {
        weight_file
            << row.segment << ","
            << row.iteration << ","
            << row.operator_idx << ","
            << operator_name_at(statistics, row.operator_idx) << ","
            << row.weight << "\n";
    }

    std::ofstream run_file(statistics_dir / "runs.csv");
    if (!run_file.is_open())
    {
        std::cerr << "Failed to open file at:" << (statistics_dir / "runs.csv").string() << "\n";
        return false;
    }

    run_file << "run,final_objective,best_found_iteration\n";
    for (const GAMRunReport& row : run_reports)
    {
        run_file
            << row.run << ","
            << row.final_objective << ","
            << row.best_found_iteration << "\n";
    }

    std::ofstream operator_file(statistics_dir / "operators.csv");
    if (!operator_file.is_open())
    {
        std::cerr << "Failed to open file at:" << (statistics_dir / "operators.csv").string() << "\n";
        return false;
    }

    operator_file
        << "operator_idx,operator_name,uses,changed_candidates,failures,"
        << "infeasible_candidates,feasible_candidates,accepted_moves,"
        << "improving_accepts,new_bests,delta_samples,delta_sum,"
        << "avg_delta,total_runtime_ms,avg_runtime_ms\n";
    for (const GAMOperatorStatistics& row : statistics.operator_stats)
    {
        const double avg_delta =
            row.delta_samples > 0
                ? (double)(row.delta_sum) / (double)(row.delta_samples)
                : 0.0;
        const double avg_runtime_ms =
            row.uses > 0
                ? row.total_runtime_ms / (double)(row.uses)
                : 0.0;

        operator_file
            << row.operator_idx << ","
            << row.operator_name << ","
            << row.uses << ","
            << row.changed_candidates << ","
            << row.failures << ","
            << row.infeasible_candidates << ","
            << row.feasible_candidates << ","
            << row.accepted_moves << ","
            << row.improving_accepts << ","
            << row.new_bests << ","
            << row.delta_samples << ","
            << row.delta_sum << ","
            << avg_delta << ","
            << row.total_runtime_ms << ","
            << avg_runtime_ms << "\n";
    }

    return true;
}

