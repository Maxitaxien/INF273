#pragma once

#include "datahandling/instance.h"

/**
 * Precomputes whether a pure drone flight launch -> customer -> land is within
 * the drone limit, ignoring truck timing and waiting.
 */
void precompute_pure_drone_feasibility(Instance &instance);

/**
 * Fast check for whether launch -> customer -> land is within the drone limit.
 *
 * Uses the precomputed cache when available and falls back to direct distance
 * evaluation otherwise.
 */
bool pure_drone_flight_within_limit(
    const Instance &instance,
    int launch_node,
    int customer,
    int land_node);
