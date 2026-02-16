if time: implement multiple overrides of fixers, perhaps in their own folder
the idea is that some of the versions are specialized to only have to scan through part of the solution

## Inserting from drone into truck

- Easiest if we just always put this at the end of the tour. Then we introduce no issues.

## Inserting from truck into drone

- We just remove the last node. So no drone infeasibility possible.
- However, the node has to be covered in a way that is feasible. This means that we use the land/launch fix strategy described below.

## Inserting from drone into other drone

- Just insert the deliver node, then use the strategy below.

## Fixing land/launch after inserting into drone:

Pre-work:

- Sort all other points in accordance to closeness to delivery target.
- Get updated indices of all points within truck tour through a linear scan, create a map.
- Create a vector of pairs, where each pair represents an occupied interval. Store in set for fast lookup:

```
struct Interval {
    int start, end;
    bool operator<(const Interval &other) const { return start < other.start; }
};

std::set<Interval> drone_intervals;

// Check overlap
auto it = drone_intervals.lower_bound({L_new, -1});
if (it != drone_intervals.end() && it->start <= R_new) overlap = true;
if (it != drone_intervals.begin() && std::prev(it)->end >= L_new) overlap = true;

// Insert
drone_intervals.insert({L_new, R_new});
```

Algorithm:

- Initiate two pointers l, r assigned to the two closest points (index 0, 1)
- Initiate a bool tracking which pointer should be moved next, r_next = true;
- For the point at each pointer, check their position in the truck sequence. Assign first to launch, second to land
- Check if the drone is in use at either of these points using interval implementation.
- If any are in use: move pointer r if r_next, else l if l_next. Check if this assignment is valid.

Break the loop if the max(launch -> delivery -> land, truck launch -> delivery) is ever larger than the drone flight limit. At this point deem the move impossible, abort and return the solution unchanged.

Improvement:
Only select launch node based on closeness. Try a new close candidate if there is an overlap. Then try to assign the landing spot to the furthest spot away that is still feasible. For instance, we could track the next launching index of the drone, and "count backwards" from there until we find something that works.

Inspiration, but not quite the idea described above:

```
// Assume delivery_idx fixed
for (int launch_idx = 0; launch_idx < delivery_idx; ++launch_idx) {
    for (int land_idx = delivery_idx+1; land_idx < truck_tour.size(); ++land_idx) {
        if (!overlaps(launch_idx, land_idx)) {
            if (max(drone_time(launch_idx, delivery_idx, land_idx),
                    truck_time(launch_idx, land_idx)) <= drone_limit) {
                assign_drone(launch_idx, delivery_idx, land_idx);
                break;
            }
        }
    }
}
```
