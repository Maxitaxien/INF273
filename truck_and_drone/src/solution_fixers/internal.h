#pragma once

#include "datahandling/instance.h"
#include "operators/interval_helpers.h"
#include "operators/route_timing.h"
#include "solution_fixers/solution_fixers.h"
#include "verification/solution.h"
#include <limits>
#include <set>

struct FlightAssignment
{
    bool feasible = false;
    int launch_idx = -1;
    int land_idx = -1;
    long long truck_wait = std::numeric_limits<long long>::max();
    long long drone_arrival = std::numeric_limits<long long>::max();
};

struct AssignmentSearchLimits
{
    int anchor_count = 0;
    int launch_back_window = 0;
    int land_forward_window = 0;
};

bool delivery_on_truck_route(const Solution &solution, int delivery);

AssignmentSearchLimits make_assignment_search_limits(
    const Instance &instance,
    int route_size,
    bool expanded);

bool flight_under_limit_with_wait(
    const Instance &instance,
    const Solution &solution,
    const RouteTiming &timing,
    int drone,
    int flight_idx);

void consider_flight_assignment(
    const Instance &instance,
    const Solution &solution,
    const RouteTiming &timing,
    const std::set<Interval> &drone_intervals,
    int delivery,
    int drone,
    int launch_idx,
    int land_idx,
    FlightAssignment &best);

FlightAssignment find_vicinity_feasible_flight(
    const Instance &instance,
    const Solution &solution,
    const RouteTiming &timing,
    const std::set<Interval> &drone_intervals,
    int delivery,
    int drone);

FlightAssignment find_two_opt_local_feasible_flight(
    const Instance &instance,
    const Solution &solution,
    const RouteTiming &timing,
    const std::set<Interval> &drone_intervals,
    const AffectedDroneFlight &affected,
    int first,
    int second);

void append_drone_flight(
    Solution &solution,
    int drone,
    int delivery,
    const FlightAssignment &assignment);
