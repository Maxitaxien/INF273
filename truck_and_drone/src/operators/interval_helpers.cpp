#include "operators/interval_helpers.h"
#include "verification/solution.h"
#include <limits>
#include <set>

std::set<Interval> get_intervals(const Solution &solution, int drone)
{
    std::set<Interval> intervals;
    const DroneCollection &collection = solution.drones[drone];
    const int flight_count = (int)(collection.launch_indices.size());

    for (int i = 0; i < flight_count; ++i)
    {
        Interval interval;
        interval.start = collection.launch_indices[i];
        interval.end = collection.land_indices[i];
        intervals.insert(interval);
    }

    return intervals;
}

bool overlaps(const std::set<Interval> &intervals, int pos1, int pos2)
{
    auto it = intervals.lower_bound({pos1, -1});

    if (it != intervals.end() && it->start <= pos2)
        return true;

    if (it != intervals.begin() && std::prev(it)->end >= pos1)
        return true;

    return false;
}

int find_containing_interval_index(const std::set<Interval> &intervals, int pos)
{
    auto it = intervals.upper_bound({pos, std::numeric_limits<int>::max()});

    if (it == intervals.begin())
        return -1;

    --it;
    if (it->start <= pos && pos <= it->end)
    {
        return std::distance(intervals.begin(), it);
    }

    return -1;
}

