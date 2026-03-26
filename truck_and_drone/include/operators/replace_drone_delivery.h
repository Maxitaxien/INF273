#pragma once

#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Replace a drone delivery with a truck delivery.
 *
 * The selected drone flight is removed and its customer is reinserted into the
 * truck route within the original launch/rendezvous interval. All admissible
 * insertions are evaluated and the best feasible full-objective move is kept.
 */
bool replace_drone_delivery(
    const Instance &instance,
    Solution &sol,
    int drone,
    int flight_idx);
