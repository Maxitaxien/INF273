#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Idea: At given truck index i, reassign at index i + 1 to the nearest neighbour
 * Pop this neighbour from it's current position, and insert the old neighbour at it's old position
 *
 * Should give quite some exploration! Perhaps we make a locally "good" decision, but mess up the later parts
 * of the solution.
 *
 * This operator only changes the truck sequence. The surrounding search
 * algorithm is expected to perform the final feasibility check.
 *
 */
bool nearest_neighbour_reassign(const Instance &inst, Solution &sol, int i);
