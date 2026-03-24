#include "operators/late_customer_drone_promotion.h"

bool late_customer_drone_promotion(
    const Instance &inst,
    Solution &sol,
    int truck_idx,
    int drone,
    int launch_window,
    int land_window)
{
    (void)inst;
    (void)sol;
    (void)truck_idx;
    (void)drone;
    (void)launch_window;
    (void)land_window;

    // TODO:
    // 1. Choose a late truck customer, ideally one with large downstream latency
    //    contribution in the Min-Sum objective.
    // 2. Remove it from the truck route and test only nearby launch/rendezvous
    //    pairs on the chosen drone.
    // 3. Score candidates by expected reduction in truck arrivals and drone wait,
    //    not by exhaustive full-route enumeration.
    // 4. Commit the best improving feasible move; otherwise restore the customer.
    return false;
}
