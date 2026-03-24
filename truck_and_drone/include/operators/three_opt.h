#pragma once

#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Stronger truck-route intensification than 2-opt.
 *
 * The intended implementation should evaluate a small subset of the classic
 * 3-opt reconnections around three breakpoints and keep the best improving move.
 * This is best used as a truck-first move, with feasibility checked by the
 * surrounding algorithm.
 */
bool three_opt(const Instance &inst, Solution &sol, int first, int second, int third);
