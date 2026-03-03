#pragma once

#include "verification/solution.h"
#include "datahandling/instance.h"
#include <set>

/**
 * Struct for drone delivery intervals for a single drone
 * Allows for easy overlap checks
 */
struct Interval
{
    int start, end;
    bool operator<(const Interval &other) const { return start < other.start || (start == other.start && end < other.end); }
};

/**
 * Get time intervals the drone in question is occupied
 */
std::set<Interval> get_intervals(const Solution &solution, int drone);

/**
 * Check if a potential new launch -> land overlaps existing drone launch -> land sequence.
 * Then, it is guaranteed to be the case not to be valid to insert it.
 * 
 * Logic is c++ black magic but efficient
 */
bool overlaps(const std::set<Interval>& intervals, int pos1, int pos2);

/**
 * Check if a idx inserted into a truck route is contained in any drone flights
 * Then, this may potentially break their feasibility.
 * 
 * Logic is c++ black magic like overlaps().  
 * @return index of interval, which should correspond to index of drone flight
 */
int find_containing_interval_index(const std::set<Interval>& intervals, int pos);