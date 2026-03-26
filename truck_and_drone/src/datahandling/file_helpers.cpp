#include "datahandling/file_helpers.h"
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;

/**
 * Private func for create_run_directory.
 */
std::string get_current_time_string()
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm *local_tm = std::localtime(&now_time);

    std::stringstream ss;
    // Replace space with '_' and ':' with '-'
    ss << std::put_time(local_tm, "%Y-%m-%d_%H:%M");
    ;
    return ss.str();
}

std::string create_algo_directory(const std::string &base, const std::string &algo_name)
{
    std::string save_dir;
    std::string dir;
    if (base == "")
    {
        save_dir = create_run_directory();
    }
    else
    {
        save_dir = base;
    }
    dir = save_dir + "/" + algo_name;
    fs::create_directories(dir);
    return dir;
}

std::string create_run_directory()
{
    std::string base = fs::current_path().parent_path().string() + "/runs/";
    std::string dir = base + "/" + get_current_time_string();
    fs::create_directories(dir);
    return dir;
}

std::string dataset_stem(const std::string &dataset)
{
    const size_t last_slash = dataset.find_last_of("/\\");
    const size_t last_dot = dataset.find_last_of(".");

    const size_t start = (last_slash == std::string::npos) ? 0 : last_slash + 1;
    const size_t length = (last_dot == std::string::npos || last_dot < start)
        ? std::string::npos
        : last_dot - start;

    return dataset.substr(start, length);
}

std::string create_dataset_statistics_directory(const std::string &run_dir, const std::string &dataset)
{
    const fs::path dir = fs::path(run_dir) / (dataset_stem(dataset) + "_statistics");
    fs::create_directories(dir);
    return dir.string();
}
