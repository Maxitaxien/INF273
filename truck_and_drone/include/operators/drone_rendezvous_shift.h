#pragma once

#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Local intensification for an existing drone flight.
 *
 * This operator keeps the assigned drone customer fixed and only shifts the
 * launch and/or rendezvous positions within a small window to reduce truck
 * waiting or improve customer arrival time.
 */
bool drone_rendezvous_shift(
    const Instance &inst,
    Solution &sol,
    int drone,
    int flight_idx,
    int launch_shift,
    int land_shift);
