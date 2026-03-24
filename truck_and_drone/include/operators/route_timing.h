#pragma once

#include "datahandling/instance.h"
#include "verification/solution.h"
#include <vector>

struct RouteTiming
{
    std::vector<long long> truck_arrival;
    std::vector<std::vector<long long>> drone_ready_at_stop;
    std::vector<long long> drone_ready_at_end;
    long long total_drone_arrival = 0;
};

RouteTiming compute_route_timing(const Instance &instance, const Solution &solution);
