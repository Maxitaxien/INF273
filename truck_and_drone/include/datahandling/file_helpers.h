#pragma once
#include <string>

/**
 * Creates directory to log results, using algorithm name.
 * In the case that multiple algorithms are to be run, base is given as the result of create_run_directory.
 * If not, base should be "", and the dir creation is handled automatically.
 */
std::string create_algo_directory(const std::string &base, const std::string &algo_name);

/**
 * Creates overall directory for this run, based on start time.
 *
 */
std::string create_run_directory();

/**
 * Returns the dataset name without directories or extension.
 */
std::string dataset_stem(const std::string &dataset);

/**
 * Creates and returns the per-dataset statistics directory inside a run.
 */
std::string create_dataset_statistics_directory(const std::string &run_dir, const std::string &dataset);
