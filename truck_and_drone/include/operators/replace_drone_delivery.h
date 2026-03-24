#pragma once

#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Replace a drone delivery with a truck delivery.
 *
 * The selected drone flight is removed and its customer is reinserted into the
 * truck route near the original launch/rendezvous interval. The move is kept
 * only if the resulting solution remains feasible.
 */
bool replace_drone_delivery(
    const Instance &instance,
    Solution &sol,
    int drone,
    int flight_idx);
