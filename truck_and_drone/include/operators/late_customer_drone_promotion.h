#pragma once

#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Lightweight truck-to-drone reassignment operator for the Min-Sum objective.
 *
 * The intended move should focus on a late truck customer with high downstream
 * impact and try a small set of nearby launch/rendezvous options on one drone,
 * instead of scanning all truck positions and all drones greedily.
 */
bool late_customer_drone_promotion(
    const Instance &inst,
    Solution &sol,
    int truck_idx,
    int drone,
    int launch_window,
    int land_window);
