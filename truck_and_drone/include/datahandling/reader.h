#pragma once
#include "instance.h"
#include <string>

/**
 * Creates a problem instance based on .txt file.
 * Expects:
 * # Header
 * (number of customers n)
 * # Header
 * (drone flight limit)
 * # Header
 * (n x n truck time matrix)
 * # Header
 * (n x n drone time matrix)
 */
Instance read_instance(const std::string&);