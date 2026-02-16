#include "operators/helpers.h"
#include "verification/solution.h"
#include "datahandling/instance.h"
#include "verification/feasibility_check.h"
#include <vector>
#include <algorithm>
#include <unordered_set>
#include <set>
#include <unordered_map>
#include <limits>

void remove_drone_flight(Solution& solution, int drone, int i) {
    solution.drones[drone].launch_indices.erase(
        solution.drones[drone].launch_indices.begin() + i
    );
    solution.drones[drone].land_indices.erase(
        solution.drones[drone].land_indices.begin() + i
    );
    solution.drones[drone].deliver_nodes.erase(
        solution.drones[drone].deliver_nodes.begin() + i
    );
}

std::pair<bool, bool> drone_landed_at_back(const Solution& solution) {
    int final_index = solution.truck_route.size() - 1;

    std::vector<bool> drone_is_invalid(2);

    for (int c = 0; c < solution.drones.size(); c++) {
        DroneCollection dc = solution.drones[c];
        for (int landing : dc.land_indices) {
            if (landing == final_index) {
                drone_is_invalid[c] = true;
            }
        }
    }
    return {drone_is_invalid[0], drone_is_invalid[1]};
}


std::vector<int> sort_points(const Instance& instance, const Solution& solution, int point)
{
    std::vector<int> points;
    std::unordered_set<int> truck_set(solution.truck_route.begin(), solution.truck_route.end());

    for (int i = 0; i < instance.n; i++) {
        if (i != point && truck_set.find(i) != truck_set.end())
        {
            points.push_back(i);
        }
    }

    // Sort indices by distance
    std::sort(points.begin(), points.end(),
              [&](int a, int b)
              {
                  return instance.drone_matrix[point][a] < instance.drone_matrix[point][b];
              });

    return points;
}

std::unordered_map<int, int> get_node_positions(const Solution& solution)
{
    std::unordered_map<int, int> positions;
    for (int i = 0; i < solution.truck_route.size(); i++)
    {
        positions[solution.truck_route[i]] = i;
    }

    return positions;
}

std::vector<long long> get_truck_arrival_time_at_index(const Instance& instance, const Solution& solution) {
    std::vector<long long> arrival_times(solution.truck_route.size());

    for (int i = 1; i < solution.truck_route.size(); i++) {
        arrival_times[i] = 
            arrival_times[i - 1] 
            + instance.truck_matrix[solution.truck_route[i - 1]][solution.truck_route[i]];
    }

    return arrival_times;
}

std::set<Interval> get_intervals(const Solution& solution, int drone)
{
    std::set<Interval> intervals;
    const DroneCollection& collection = solution.drones[drone];

    for (int i = 0; i < collection.launch_indices.size(); i++)
    {
        Interval interval;
        interval.start = collection.launch_indices[i];
        interval.end = collection.land_indices[i];
        intervals.insert(interval);
    }

    return intervals;
}

bool overlaps(const std::set<Interval>& intervals, int pos1, int pos2) {
    auto it = intervals.lower_bound({pos1, -1});

    if (it != intervals.end() && it->start <= pos2)
        return true;

    if (it != intervals.begin() && std::prev(it)->end >= pos1)
        return true;

    return false;
}

int find_containing_interval_index(const std::set<Interval>& intervals, int pos) {
    // Find the first interval whose start is > pos
    auto it = intervals.upper_bound({pos, std::numeric_limits<int>::max()});

    if (it == intervals.begin()) return -1; // all intervals start after pos

    --it; // candidate interval
    if (it->start <= pos && pos <= it->end) {
        return std::distance(intervals.begin(), it); // index of interval
    }

    return -1; // not found
}

std::pair<bool, Solution> assign_launch_and_land(const Instance& instance, Solution& solution, int new_deliver, int drone)
{
    std::unordered_map<int, int> node_positions = get_node_positions(solution);
    std::vector<int> sorted_points = sort_points(instance, solution, new_deliver);
    std::set<Interval> drone_intervals = get_intervals(solution, drone);
    std::vector<long long> truck_arrival_times = get_truck_arrival_time_at_index(instance, solution);

    // Two pointer for assignment
    if (sorted_points.size() < 2)
    return {false, solution};

    for (int l = 0; l < sorted_points.size(); l++) {
        for (int r = l + 1; r < sorted_points.size(); r++) {

            int p1 = sorted_points[l];
            int p2 = sorted_points[r];

            int pos1 = node_positions.at(p1);
            int pos2 = node_positions.at(p2);
            if (pos1 > pos2) std::swap(pos1, pos2);

            long long flying_distance = std::max
            (
                instance.drone_matrix[p1][new_deliver] +
                instance.drone_matrix[new_deliver][p2],
                truck_arrival_times[pos2] - truck_arrival_times[pos1]
            );

            bool overlap = overlaps(drone_intervals, pos1, pos2);

            if (flying_distance <= instance.lim && !overlap) {
                solution.drones[drone].launch_indices.push_back(pos1);
                solution.drones[drone].deliver_nodes.push_back(new_deliver);
                solution.drones[drone].land_indices.push_back(pos2);
                return {true, solution};
            }
        }
    }

    return {false, solution};
}

Solution fix_feasibility_for_drone(const Instance& instance, Solution& solution, int drone) {
    DroneCollection& c = solution.drones[drone];
    std::set<Interval> intervals = get_intervals(solution, drone);

    for (int i = 0; i < c.launch_indices.size(); ) {
        int launch_idx = c.launch_indices[i];
        int land_idx   = c.land_indices[i];
        int deliver    = c.deliver_nodes[i];

        bool bad = false;

        if (launch_idx >= land_idx)
            bad = true;

        if (!specific_drone_flight_under_lim(instance, solution, drone, i))
            bad = true;

        // Rebuild intervals each iteration (minimal correctness)
        std::set<Interval> intervals = get_intervals(solution, drone);

        Interval self{launch_idx, land_idx};
        auto it = intervals.find(self);
        if (it != intervals.end())
            intervals.erase(it);

        if (overlaps(intervals, launch_idx, land_idx))
            bad = true;

        if (bad) {
            auto fix = assign_launch_and_land(instance, solution, deliver, drone);
            if (!fix.first) {
                remove_drone_flight(solution, drone, i);

                int truck_pos = launch_idx + 1;
                solution.truck_route.insert(solution.truck_route.begin() + truck_pos, deliver);

                continue; // vector shifted
            }
        }

        i++;
    }

    return solution;
}

Solution fix_overall_feasibility(const Instance& instance, Solution& solution) {
    solution = fix_feasibility_for_drone(instance, solution, 0);
    solution = fix_feasibility_for_drone(instance, solution, 1);
    solution = simple_fix_validity(solution);
    return solution;
}

Solution simple_fix_validity(Solution& solution) {
    int final_index = solution.truck_route.size() - 1;
    std::vector<int> to_reinsert;

    for (int c = 0; c < solution.drones.size(); c++) {
        DroneCollection& dc = solution.drones[c]; 
        
        for (int i = 0; i < dc.deliver_nodes.size(); ) {
            int landing = dc.land_indices[i];
            if (landing == final_index) {
                int deliver = dc.deliver_nodes[i];
                to_reinsert.push_back(deliver);

                dc.launch_indices.erase(dc.launch_indices.begin() + i);
                dc.land_indices.erase(dc.land_indices.begin() + i);
                dc.deliver_nodes.erase(dc.deliver_nodes.begin() + i);
            } else {
                i++;
            }
        }
}
    
    for (int reinsert : to_reinsert) {
        solution.truck_route.push_back(reinsert);
    }

    return solution;
}

Solution fix_validity(const Instance& instance, Solution& solution, int drone) {
    return solution;
}