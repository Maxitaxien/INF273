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

/**
 * Randomized nearest-neighbour constructor that samples from a short list of the
 * currently closest truck candidates using exponential rank bias.
 *
 * This keeps the same truck-only scaffold as the deterministic NN, but introduces
 * controlled diversification for restart paths.
 */
Solution roulette_nearest_neighbour(const Instance& problem_instance);
