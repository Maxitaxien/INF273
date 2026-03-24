#pragma once

#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Paper-inspired move targeting truck customers that currently serve as launch
 * or rendezvous nodes for drone flights.
 *
 * The main idea is to relocate one such truck node within the truck route and
 * then re-time or re-plan only the affected drone flights instead of rebuilding
 * the whole solution from scratch.
 */
bool origin_destination_relocation(
    const Instance &inst,
    Solution &sol,
    int truck_idx,
    int insert_after_idx);
