#include "datahandling/save_to_csv.h"

#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>

#include <filesystem>

std::string BASE = std::filesystem::current_path().parent_path().string() + "/runs/";

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
    ss << std::put_time(local_tm, "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

bool save_to_csv(
    std::string algo_name,
    std::string dataset, 
    long double avg, 
    long long best, 
    double improvement_percent, 
    long double avg_runtime
) {
    std::string file_path = BASE + clean_dataset(dataset) + "_" + get_current_time_string() + ".csv";
    std::ofstream file(file_path);

    if (!file.is_open()) {
        std::cerr << "Failed to open file at:" << file_path << "\n";
        return false;
    }

    file << clean_dataset(dataset) << ",,,,\n";
    file << " ,Average objective,Best Objective,Improvement (%),Average running time (in seconds)\n";
    file << algo_name << "," << avg << "," << best << "," << improvement_percent << "," << avg_runtime << "\n";

    return true;
}