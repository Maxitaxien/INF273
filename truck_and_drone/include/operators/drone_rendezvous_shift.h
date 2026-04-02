#pragma once

#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Local intensification for an existing drone flight.
 *
 * This operator keeps the assigned drone customer fixed and searches new
 * launch and rendezvous indices inside bounded windows around the current
 * flight. It only considers non-overlapping assignments for the same drone
 * and commits the best feasible move within that window.
 */
bool drone_rendezvous_shift(
    const Instance &inst,
    Solution &sol,
    int drone,
    int flight_idx,
    int launch_window,
    int land_window);
