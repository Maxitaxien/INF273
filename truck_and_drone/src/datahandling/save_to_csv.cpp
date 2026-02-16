#include "datahandling/save_to_csv.h"

#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>

#include <filesystem>

namespace fs = std::filesystem;

std::string BASE = fs::current_path().parent_path().string() + "/runs/";

std::string clean_dataset(const std::string& dataset) {
    size_t last_slash = dataset.find_last_of("/\\"); 
    size_t last_dot = dataset.find_last_of(".");
    
    size_t start = (last_slash == std::string::npos) ? 0 : last_slash + 1;
    size_t length = (last_dot == std::string::npos || last_dot < start) ? std::string::npos : last_dot - start;
    
    return dataset.substr(start, length);
}

std::string get_current_time_string() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm* local_tm = std::localtime(&now_time);

    std::stringstream ss;
    // Replace space with '_' and ':' with '-'
    ss << std::put_time(local_tm, "%d-%m-%Y_%H");;
    return ss.str();
}

bool save_to_csv(
    std::string algo_name,
    std::string dataset, 
    long double avg, 
    long long best, 
    double improvement_percent, 
    long double avg_runtime,
    std::string solution_str
) {
    std::string DIR = BASE + get_current_time_string();

    if (fs::create_directory(DIR)) {
        std::cout << "Directory '" << DIR << "' created successfully.\n";
    } else {
        std::cout << "Directory '" << DIR << "' already exists or failed to create.\n";
    }

    std::string csv_path = DIR + "/" + clean_dataset(dataset) + ".csv";
    std::ofstream csvfile(csv_path);

    std::string solution_path = DIR + "/" + clean_dataset(dataset) + ".txt";
    std::ofstream solutionfile(solution_path);

    if (!csvfile.is_open()) {
        std::cerr << "Failed to open file at:" << csv_path << "\n";
        return false;
    }

    csvfile << clean_dataset(dataset) << ",,,,\n";
    csvfile << " ,Average objective,Best Objective,Improvement (%),Average running time (in seconds)\n";
    csvfile << algo_name << "," << avg << "," << best << "," << improvement_percent << "," << avg_runtime << "\n";

    if (!solutionfile.is_open()) {
        std::cerr << "Failed to open file at:" << solution_path << "\n";
        return false;
    }

    solutionfile << solution_str << "\n";

    return true;
}