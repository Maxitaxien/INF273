#pragma once

#include "verification/solution.h"

/**
 * Substitute a truck delivery with a drone delivery
 * Effect: Reduces truck route size by one. Assigns a drone to deliver to the popped node.
 * NB: Should only be run if no existing drones launch or land at the delivery.
 *
 * All drones launching/landing indexes after the popped entry have their values decreased by 1.
 * All drones with a flight overlapping the popped index potentially need reassignment - they may now be infeasible
 *
 * @param idx: index from the truck route to replace
 * @param drone: 0 or 1, indicating which drone we replace with
 *
 */
bool substitute_truck_delivery(Solution &sol, int idx, int drone);