#pragma once

#include "verification/solution.h"
#include<string>


/*
 * Converts solutions to submission format to input into website.
 */
std::string convert_to_submission(const Solution& solution);