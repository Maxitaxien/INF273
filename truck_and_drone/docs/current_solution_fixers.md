# Current Solution Fixers

This is the authoritative reference for the current shared repair and fixer logic used by the active operator mix.

## Purpose and Scope

- This document covers the current core repair stack, not every secondary helper in the repository.
- The focus is the shared machinery that operators rely on after mutating the truck route or the drone schedule.
- That includes:
  - collecting flights affected by a move
  - localized repair of only the affected flights
  - greedy assignment of a new drone launch/land pair
  - per-drone cleanup and full-solution fallback repair
  - the greedy destroy/repair insertion logic used by Shaw-style operators
- For the operator-side context of the destroy/repair move itself, see [current_mix_operators.md](current_mix_operators.md).

## Core Data Structures and Shared Concepts

### `AffectedDroneFlight`

- `AffectedDroneFlight` is the shared record used to carry a flight through a repair pipeline.
- It stores:
  - the drone id
  - the flight index inside that drone
  - the delivered customer
  - the original launch index
  - the original land index
- The key idea is that repairs track flights by both customer identity and old anchor positions, not just by raw array index.

### `FlightAssignment`

- `FlightAssignment` is the scoring record used while searching for a feasible new launch/land pair.
- It stores:
  - whether a candidate is feasible
  - the chosen launch and land indices
  - downstream delay impact
  - truck wait
  - drone wait
  - drone arrival time
- The search logic compares feasible candidates lexicographically through those timing-derived fields rather than by objective value directly.

### `AssignmentSearchLimits`

- `AssignmentSearchLimits` controls how aggressively the vicinity search expands around promising truck anchors.
- It stores:
  - how many candidate anchor positions to try
  - how far to look backward for launch
  - how far to look forward for land
- `make_assignment_search_limits(...)` provides two passes:
  - a smaller first pass
  - a larger expanded pass if the first one finds nothing

### Route timing and overlap checks

- The fixer layer uses `compute_route_timing(...)` or `compute_route_timing_from_canonical_solution(...)` to evaluate synchronized truck/drone timing.
- Interval overlap is screened with `get_intervals(...)` and `overlaps(...)`.
- A feasible flight is not just one that fits the raw drone distance limit:
  - it must also fit with the actual truck arrival times and existing flights on the same drone
  - the waiting implied by the chosen rendezvous must still respect `instance.lim`

### Terminal-depot landing canonicalization

- Whole-candidate evaluation uses `canonicalize_terminal_depot_landings(...)`.
- That canonicalization allows the last feasible drone flight to land at the implicit return-to-depot position when appropriate.
- The local assignment helpers now also generate `land_idx == truck_route.size()` directly as a terminal-depot landing candidate when that is a better local repair than landing on an explicit truck stop.
- The paired checker `canonical_drone_schedule_consistent(...)` then enforces:
  - matching flight-array sizes
  - valid launch/land index ranges
  - strictly increasing launch-to-land intervals
  - no interval overlap within a drone
  - at most one terminal-depot landing, and only on the logically last flight

### Normalized vs assume-valid entry points

- Several fixer entry points come in two forms:
  - a normalized version
  - an `assume_valid` version
- The normalized version first runs `simple_fix_validity(...)` to clean stale or obviously invalid structure.
- The `assume_valid` version skips that normalization so hot local operators can stay cheaper when they already preserve the structural invariants they care about.
- This distinction matters for:
  - `greedy_assign_launch_and_land(...)` vs `greedy_assign_launch_and_land_assume_valid(...)`
  - `assign_launch_and_land_n_lookahead(...)` vs `assign_launch_and_land_n_lookahead_assume_valid(...)`

## Collecting Flights Affected by a Move

### `collect_affected_drone_flights_if(...)`

- This is the shared internal collector used by the move-specific front-ends.
- It iterates over all drones, then over the aligned prefix of each drone's:
  - `launch_indices`
  - `land_indices`
  - `deliver_nodes`
- A flight is collected only if the caller-supplied predicate on `(launch_idx, land_idx)` returns true.
- Each collected entry becomes an `AffectedDroneFlight` with the flight's original customer and anchors preserved.

### `collect_two_opt_affected_drone_flights(...)`

- This is the specialized collector for a 2-opt reversal on truck indices `first` and `second`.
- It defines the reversed segment as `[first + 1, second]`.
- A drone flight is considered affected if:
  - `launch_idx <= second`
  - `land_idx >= first + 1`
- In other words, any flight whose interval intersects the reversed corridor is collected, even if only part of the interval crosses it.

### `collect_swap_affected_drone_flights(...)`

- This is the collector for a truck-node swap.
- It takes the min/max of the two swapped indices and treats that whole corridor conservatively.
- A flight is collected if:
  - `launch_idx <= right`
  - `land_idx >= left`
- That means a flight is marked affected whenever it spans across, starts inside, or ends inside the swap corridor.

### `collect_removed_anchor_drone_flights(...)`

- This is the narrow collector for removing one truck anchor from the route.
- A flight is collected only if:
  - `launch_idx == removed_idx`
  - or `land_idx == removed_idx`
- This is used when the operator knows the mutation is a single removed truck stop rather than a broader corridor change.

### `remap_drone_anchor_indices_by_node(...)`

- This helper is the first stabilization step after a truck-route permutation.
- It builds a map from truck node id to its new index in the mutated route.
- Then, for each aligned flight entry that exists in both the before and after solutions, it:
  - reads the old launch node and old land node from the pre-move truck route
  - looks up their new positions in the candidate route
  - rewrites the candidate flight anchors if both nodes still exist and the new launch is still before the new land
- If an anchor node disappeared or would become inverted, the helper leaves that flight unchanged so later repair can decide what to do.
- Flights within a drone are sorted afterward when more than one flight exists.

## Localized Repair of Affected Flights

### `repair_affected_drone_flights_localized(...)`

- This is the main shared localized repair entry point.
- It rebuilds only the flights marked as affected and keeps unaffected flights frozen.

### Step-by-step flow

1. Start from the operator-mutated `candidate` and the original `before_move`.
2. Remap drone anchors by node identity with `remap_drone_anchor_indices_by_node(...)`.
3. Run an early feasibility check on the remapped candidate.
4. If the remapped candidate already passes, accept it immediately without rebuilding anything.
5. If the candidate still fails and there are no affected flights to repair, fail.
6. Copy the remapped candidate into a working solution.
7. Sort the affected flights in descending `(drone, flight_idx)` order.
8. Remove those flights from the working solution in that descending order.
9. Sort the same logical affected flights in ascending `(launch_idx, land_idx, drone, flight_idx)` order.
10. Reinsert each removed delivery on its original drone using the assignment selector for this repair path.
11. Sort any drone schedule that was touched and still contains multiple flights.
12. Validate the repaired working solution.
13. If it passes, return the repaired candidate. Otherwise fail and leave the caller with the remapped candidate as the unsuccessful fallback state.

### Why removal is descending and reinsertion is ascending

- Removal is done in descending `(drone, flight_idx)` order so array erases do not invalidate later indices that still need to be removed from the same drone.
- Reinsertion is done in ascending geometric order so the rebuilt flights are added back in a consistent corridor order rather than arbitrary input order.

### How unaffected flights are treated

- Unaffected flights are not rebuilt.
- They survive exactly as remapped-by-node if their anchor nodes still exist and still form a valid forward interval.
- This is what makes the repair localized rather than a full drone replanning pass.

### How reassignment works in the localized path

- `repair_affected_drone_flights_localized(...)` uses `find_vicinity_feasible_flight(...)` as its assignment selector.
- That means each removed delivery is reassigned on its original drone, not on any drone.
- The search is local and timing-aware:
  - it uses current route timing
  - it respects same-drone interval overlap
  - it rejects assignments whose wait-adjusted duration exceeds the drone limit

### The `allowed_missing` validation path

- Some callers temporarily allow a delivery to be missing during repair bookkeeping.
- When `allowed_missing` is non-empty, the internal validator does not check the working solution as-is.
- Instead it creates a `completed` copy for validation:
  - if an allowed-missing delivery is already on the truck route, it leaves it alone
  - if it is already delivered by some drone, it leaves it alone
  - otherwise it appends that delivery to the truck route exactly for the validation call
- This lets a move validate "everything except the still-to-be-reinserted customer" without pretending the delivery vanished permanently.

### `repair_after_two_opt_localized(...)`

- `repair_after_two_opt_localized(...)` uses the same internal repair engine but swaps the assignment selector.
- Instead of immediately using generic vicinity search, it calls `find_two_opt_local_feasible_flight(...)`.
- That selector first searches a corridor centered on the reversed 2-opt region plus a small padding, including terminal-depot landing as a candidate when `land_idx == route_size` is legal.
- If it still finds nothing, it falls back to the normal vicinity search.

## Greedy Launch/Land Assignment

### `greedy_assign_launch_and_land(...)`

- This is the normalized entry point.
- It first runs `simple_fix_validity(...)`.
- Then it searches for a feasible launch/land pair for a new drone delivery on the requested drone.

### `greedy_assign_launch_and_land_assume_valid(...)`

- This is the faster hot-path variant.
- It skips the initial normalization and assumes the caller already kept the solution structurally sane enough for local assignment search.

### Shared search flow

- Both functions delegate to the same implementation.
- They immediately reject:
  - invalid drone ids
  - a delivery that is already on the truck route
- Then they compute:
  - current route timing
  - the current interval set of the target drone
- The primary search is `find_vicinity_feasible_flight(...)`.

### Candidate anchor generation

- The vicinity search starts from a ranked anchor list, not from the full route in arbitrary order.
- It always includes depot index `0` first.
- Then it adds truck stops ranked by drone-distance-to-delivery via `sort_by_distance_to_point_drone(...)`.
- Each truck stop is converted to its current route index through `get_customer_positions(...)`.
- Duplicate anchor positions are removed.
- The primary vicinity pass still skips the final explicit truck position as an anchor.
- Terminal-depot landing is handled separately as an extra landing option, and the later lookahead fallback may still launch from the final explicit truck stop because it can land at the implicit depot.

### Two search passes from `make_assignment_search_limits(...)`

- The vicinity search runs in two passes:
  - a tighter first pass
  - a larger expanded pass if needed
- First pass limits:
  - up to `6` anchor positions
  - up to `2` launch positions backward from the anchor
  - up to `max(3, n / 20)`, capped at `6`, forward for land
- Expanded pass limits:
  - up to `10` anchor positions
  - up to `4` launch positions backward
  - up to `max(6, n / 10)`, capped at `10`, forward for land
- For each anchor, the search tries:
  - launches from the bounded backward window
  - lands from `max(launch + 1, anchor)` forward through the bounded land window
- In both passes, the search also considers `land_idx == route_size` as a terminal-depot landing candidate when the interval and timing checks allow it.

### Scoring and tiebreaks in `consider_flight_assignment(...)`

- A candidate is screened out immediately if:
  - the route is too short
  - indices are out of bounds
  - launch is not before land
  - the interval overlaps an existing flight on that drone
  - the raw drone leg is not in the pure-drone feasibility cache
  - the wait-adjusted round trip exceeds the drone limit
- Remaining candidates are ranked lexicographically by:
  1. lower downstream delay impact
  2. lower truck wait
  3. lower drone wait
  4. lower drone arrival time
  5. lower land index
- For terminal-depot landings specifically, the scorer treats:
  - `land_node = 0`
  - `drone_wait = 0`
  - `truck_wait = 0`
  - `downstream_delay_impact = 0`
- The key idea is to prefer assignments that disturb future truck progress and drone synchronization as little as possible.

### Fallback when vicinity search fails

- If `find_vicinity_feasible_flight(...)` returns no feasible assignment, the greedy assigner falls back to a bounded lookahead strategy.
- It takes up to `4` nearest launch candidates from the distance-ranked truck stops.
- For each one it calls `assign_launch_and_land_n_lookahead_impl(...)` with:
  - the chosen launch index
  - a lookahead of `min(route_size - 1, max(3, n / 10))`
- That helper scans forward only, stopping before the next already-scheduled launch on the same drone if needed.
- In the fallback path, the launch candidate set may include the final explicit truck stop, because terminal-depot landing is now a valid forward endpoint.
- It uses a similar timing-based tiebreak:
  - downstream delay impact
  - truck wait
  - drone wait
  - earlier land index
- It can also choose `land_idx == route_size`, meaning the flight returns to the implicit final depot instead of an explicit truck stop.
- Among successful fallback assignments, the greedy assigner keeps the one with the best full objective value.

## Per-Drone Repair and Global Feasibility Repair

### `simple_fix_validity(...)`

- This is the cheapest normalizer in the repair stack.
- It does not try to optimize anything.
- Its job is to remove obviously stale or invalid drone flights and make sure uncovered customers are returned to the truck exactly once.

### What `simple_fix_validity(...)` removes

- It removes a drone flight if any of these hold:
  - invalid launch or land indices
  - launch not before land
  - non-terminal land at or beyond the final truck customer index
  - terminal-depot landing used by any flight other than the logically last flight on that drone
  - duplicate customer already on the truck
  - duplicate customer already kept by another drone flight
- When it removes such a flight, it queues the delivered customer for truck reinsertion unless that customer is already known to be covered elsewhere.

### How `simple_fix_validity(...)` reinserts customers

- Reinserted customers are appended to the truck route exactly once.
- It uses sets to avoid re-adding the same customer multiple times.

### `fix_feasibility_for_drone(...)`

- This is a targeted per-drone cleanup pass.
- It walks one drone's flight list and recomputes timing each iteration.
- A flight is considered bad if:
  - `flight_under_limit_with_wait(...)` fails
  - or its interval overlaps another flight on the same drone
  - or launch is not before land
- Bad flights are removed and then immediately retried on the same drone with `greedy_assign_launch_and_land_assume_valid(...)`.
- If reassignment fails, the customer is inserted back onto the truck near `launch_idx + 1`.
- After the pass, the drone schedule is sorted if it still has multiple flights.

### `fix_overall_feasibility(...)`

- This is the broader fallback used after route mutations when localized repair is not enough.
- It keeps the truck route fixed and tries progressively heavier drone repair.

### Actual fallback order in `fix_overall_feasibility(...)`

1. Run `simple_fix_validity(...)`.
2. If `master_check(...)` now passes, return immediately.
3. Call full `drone_planner(...)`.
4. If the planned solution passes `master_check(...)`, return it.
5. Otherwise copy the normalized solution into a fallback working solution.
6. Run `fix_feasibility_for_drone(...)` for each drone in that fallback.
7. Finish with `simple_fix_validity(...)`.
8. Return the fallback result even if it is only a best-effort cleanup and not guaranteed globally feasible.

## Greedy Repair Used by Destroy/Repair Operators

- The current destroy/repair operators such as Shaw use a separate greedy repair loop in `src/operators/shaw_removal_greedy_repair.cpp`.
- This sits outside `solution_fixers/`, but it uses the same core feasibility ideas and should be understood alongside the fixer stack.
- For the operator-level destroy side, see the Shaw section in [current_mix_operators.md](current_mix_operators.md).

### `greedy_repair_one_customer(...)`

- This function tries to repair one missing customer into a partially destroyed solution.
- It explores two families of insertions:
  - truck insertion
  - direct drone flight insertion

### Truck insertion branch

- It tries every truck insertion position from `1` through the current route size.
- Each truck insertion uses `insert_truck_customer_and_shift_drones(...)`.
- That means every downstream launch or land index at or after the insertion point is incremented immediately.
- The resulting full candidate is evaluated with `evaluate_candidate_with_timing(...)`.

### Drone insertion branch

- It tries every launch index, every forward land index, and every drone.
- Before creating a full candidate, it pre-screens the raw flight with `pure_drone_flight_within_limit(...)`.
- If the raw flight passes that precheck, it appends the direct flight and then evaluates the full candidate with `evaluate_candidate_with_timing(...)`.

### Full-candidate validation

- `evaluate_candidate_with_timing(...)` does more than a simple `master_check(...)`.
- It:
  - canonicalizes terminal-depot landings
  - checks internal drone schedule consistency
  - recomputes synchronized route timing
  - verifies each drone flight against the actual wait-adjusted limit
  - computes the objective from the resulting timing
- Only candidates that survive that full timing-aware validation are eligible.

### `greedy_repair(...)`

- `greedy_repair(...)` works on a whole repair pool, not just one customer.
- It first randomizes the repair pool order with `random_shuffle(...)`.
- That shuffle only affects the order in which customers are considered inside a round.
- Each round still performs a global best-next-customer choice:
  - try `greedy_repair_one_customer(...)` for every remaining customer
  - re-evaluate the repaired trial with `evaluate_candidate_with_timing(...)`
  - keep the best feasible result across all remaining customers
- Then it commits only that best customer insertion and removes that customer from the pool.
- So the process is randomized at the candidate-ordering level, but still greedy at the accepted-step level.

## Key Code Paths

- Public repair/fixer surface:
  - `include/solution_fixers/solution_fixers.h`
- Core assignment and timing helpers:
  - `src/solution_fixers/helper_methods.cpp`
  - `src/solution_fixers/launch_land_assignment.cpp`
- Localized affected-flight repair:
  - `src/solution_fixers/two_opt_fixers.cpp`
- Cheap and fallback feasibility repair:
  - `src/solution_fixers/simple_feasibility_fixers.cpp`
  - `src/solution_fixers/drone_planner_fixers.cpp`
- Destroy/repair operator-side greedy insertion:
  - `src/operators/shaw_removal_greedy_repair.cpp`
  - `src/operators/candidate_evaluation.cpp`
