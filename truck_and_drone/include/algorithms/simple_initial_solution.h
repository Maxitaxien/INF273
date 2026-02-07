#pragma once
#include "verification/solution.h"

/**
 * Generates a solution where all customers are served in the order 1 -> n by truck.
 * Drones are unused.
 */
Solution simple_initial_solution(int n);