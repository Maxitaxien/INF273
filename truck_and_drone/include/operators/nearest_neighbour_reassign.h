#pragma once
#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Idea: At given truck index i, reassign at index i + 1 to the nearest neighbour
 * Pop this neighbour from it's current position, and insert the old neighbour at it's old position
 *
 * Reject the move if the resulting schedule becomes infeasible.
 *
 *
 * Should give quite some exploration! Perhaps we make a locally "good" decision, but mess up the later parts
 * of the solution.
 *
 * TODO: Evauate if this can potentially also take points that are covered by drone. A bit more complexity though
 *
 */
bool nearest_neighbour_reassign(const Instance &inst, Solution &sol, int i);
