#pragma once
#include <string>

/**
 * Creates directory to log results, using algorithm name and current time.
 */
std::string create_run_directory(const std::string& algo_name);