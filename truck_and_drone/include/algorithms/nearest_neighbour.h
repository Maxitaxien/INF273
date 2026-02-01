#pragma once
#include <vector>
#include "verification/solution.h"
#include "datahandling/instance.h"

/**
 * Uses the nearest neighbour heuristic to construct a solution consisting solely of
 * a permutation of nodes for the truck route, leaving drone uses empty to be augmented later.
 * 
 * @param problem_instance The instance for which to create the tour
 * @return Solution with truck route and no drone usage
 */
Solution nearest_neighbour(const Instance& problem_instance);