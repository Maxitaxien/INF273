#pragma once

#include "datahandling/instance.h"
#include "verification/solution.h"

#include <vector>

void add_unique_int(std::vector<int> &values, int value);

bool canonical_drone_schedule_consistent(const Solution &solution);

bool evaluate_candidate_with_timing(
    const Instance &instance,
    Solution candidate,
    long long &cost,
    Solution &canonical_candidate);

void insert_truck_customer_and_shift_drones(
    Solution &solution,
    int insert_pos,
    int customer);

void append_direct_drone_flight(
    Solution &solution,
    int drone,
    int launch_idx,
    int customer,
    int land_idx);
