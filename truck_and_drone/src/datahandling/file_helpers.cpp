#include "datahandling/file_helpers.h"
#include <chrono>
#include <sstream>
#include <filesystem>

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