# Current Mix Operators

This is the authoritative reference for the six operators used in the current GAM mix defined in `src/main.cpp`.

## NN-Reassign

### Purpose / neighborhood
- Swaps one truck customer with a nearby truck customer already present elsewhere in the current solution.
- The move is meant to make a locally attractive truck choice while disturbing the later route enough to create exploration.

### How the candidate is generated
- `nearest_neighbour_reassign_random` samples a truck index `i` uniformly from valid non-depot positions that still have a successor.
- `nearest_neighbour_reassign` looks up the truck customer at `truck_route[i]`.
- It ranks currently truck-served customers by truck distance using `sort_by_distance_to_point_truck(...)`.
- It picks the closest candidate, unless that is already the immediate successor at `i + 1`, in which case it tries the second-closest option.
- It then swaps the selected customer into position `i`.

### What happens if feasibility breaks
- The raw swap can invalidate drone flights whose anchors cross one of the swapped truck positions.

### What repair path is used
- It calls `collect_swap_affected_drone_flights(...)` on the original solution.
- It first tries `repair_affected_drone_flights_localized(...)` to repair only the impacted flights.
- If that fails, it falls back to `fix_overall_feasibility(...)`.
- The move is rejected if `master_check(...)` still fails afterward.

### What screening, bounding, or shortlist tricks it uses
- It never considers depot position `0`.
- It only looks at customers already on the truck route.
- It avoids the trivial "swap with the already-following customer" case when a second-nearest option exists.

### What makes it return `false`
- Route too short.
- Invalid chosen index.
- No nearest-neighbor candidate.
- Chosen customer cannot be found on the route.
- Localized repair and full repair both fail feasibility.

### Key helper / code paths
- `src/operators/nearest_neighbour_reassign.cpp`
- `sort_by_distance_to_point_truck(...)`
- `collect_swap_affected_drone_flights(...)`
- `repair_affected_drone_flights_localized(...)`
- `fix_overall_feasibility(...)`

## Two-Opt Arrival Screened

### Purpose / neighborhood
- Applies a 2-opt reversal on the truck route, but only for candidates that look promising under a truck-arrival surrogate.
- This is the current mix's faster truck-improvement move.

### How the candidate is generated
- `two_opt_arrival_screened` enumerates all legal `(first, second)` 2-opt boundaries.
- For each pair, it computes the truck-only arrival-sum score on the reversed route using `truck_arrival_sum_only(...)`.
- It keeps only moves whose truck-arrival score is strictly better than the current truck-arrival score.
- Those promising moves are ranked by:
  1. lower truck-arrival score,
  2. fewer affected drone flights,
  3. lower `first`,
  4. lower `second`.
- It then tries only a tiny shortlist:
  - `2` moves when `n >= 50`
  - `3` moves otherwise
- The shortlist trial order is randomized with `roulette_wheel_selection_exponential(...)`, so the best-ranked move is favored but not forced.

### What happens if feasibility breaks
- Reversing a truck segment can invalidate launch/land indices and flight orderings for drones crossing that segment.

### What repair path is used
- `two_opt(...)` first applies the route reversal.
- If the candidate fails `master_check(...)`, it tries `repair_after_two_opt_localized(...)`.
- If that fails, it falls back to `fix_overall_feasibility(...)`.
- A candidate is rejected if feasibility still fails afterward.

### What screening, bounding, or shortlist tricks it uses
- Heavy screening happens before any full feasibility repair:
  - truck-arrival surrogate filter,
  - affected-flight count as a tiebreak,
  - shortlist cap of `2` or `3`,
  - exponential roulette over only the shortlist.
- This is the main reason it is cheaper than trying every feasible 2-opt fully.

### What makes it return `false`
- Route too short.
- No truck-arrival-improving reversal exists.
- None of the shortlisted moves can be repaired into a feasible solution.

### Key helper / code paths
- `src/operators/two_opt.cpp`
- `two_opt(...)`
- `truck_arrival_sum_only(...)`
- `count_affected_drone_flights(...)`
- `repair_after_two_opt_localized(...)`
- `fix_overall_feasibility(...)`

## Truck Replacement Greedy

### Purpose / neighborhood
- Converts one truck-served customer into a drone-served customer.
- This is the truck-to-drone promotion move in the current mix.

### How the candidate is generated
- `replace_truck_delivery_greedy` enumerates every pair of:
  - truck customer position `i > 0`
  - drone id
- `replace_truck_delivery(...)` removes the truck customer from the route.
- It collects flights that depended on the removed truck anchor with `collect_removed_anchor_drone_flights(...)`.
- It repairs those affected flights first.
- It then tries to reassign the removed customer as a drone delivery on the selected drone using `greedy_assign_launch_and_land_assume_valid(...)`.

### What happens if feasibility breaks
- Removing a truck anchor can invalidate drone flights that launched from, landed on, or otherwise depended on that stop.

### What repair path is used
- It first calls `repair_affected_drone_flights_localized(...)` with the removed customer in the repair list.
- Only after the local anchor repair succeeds does it attempt the new drone assignment.
- The final candidate must pass `master_check(...)`.

### What screening, bounding, or shortlist tricks it uses
- The greedy wrapper keeps only the top `5` feasible candidates by objective value.
- It then chooses from that top-5 list using `roulette_wheel_selection_exponential(...)` instead of always taking the single best.
- This preserves some diversification while staying focused on good moves.

### What makes it return `false`
- Invalid truck index.
- Invalid drone id.
- Anchor-flight repair fails.
- No feasible launch/land assignment exists for the promoted customer.
- Final `master_check(...)` fails.
- The greedy wrapper also returns `false` if no feasible candidate exists at all.

### Key helper / code paths
- `src/operators/replace_truck_delivery.cpp`
- `collect_removed_anchor_drone_flights(...)`
- `repair_affected_drone_flights_localized(...)`
- `greedy_assign_launch_and_land_assume_valid(...)`

## Targeted drone-to-truck

### Purpose / neighborhood
- Converts one existing drone-served customer back onto the truck route.
- This is the current mix's targeted drone demotion move.

### How the candidate is generated
- It first ranks existing drone flights with `rank_drone_demotion_candidates(...)`.
- The ranking favors flights that currently create a lot of downstream truck delay:
  1. higher `downstream_delay_impact`
  2. higher immediate `truck_wait`
  3. earlier `land_idx`
  4. longer drone duration
  5. smaller span
  6. lower drone id / flight index
- It then evaluates only a shortlist:
  - top `3` ranked flights when `n >= 50`
  - top `4` otherwise
- For each shortlisted flight, `replace_drone_delivery_with_bounds(...)` removes the flight and tries every truck insertion position in the allowed range, keeping the best feasible reinsertion.

### What happens if feasibility breaks
- Removing the flight itself is easy; the risk is that reinserting the customer on the truck shifts later truck indices and breaks other drone anchors.

### What repair path is used
- There is no separate repair stage.
- Instead, each truck insertion candidate immediately shifts downstream drone indices with `shift_drone_indices_after_truck_insert(...)` and is kept only if it passes `master_check(...)`.

### What screening, bounding, or shortlist tricks it uses
- The operator avoids enumerating every flight uniformly.
- It ranks flights by timing-derived harm and evaluates only the top `3` or `4`.
- Within each chosen flight, it keeps only the best feasible truck reinsertion.
- Among feasible shortlisted results it sorts by objective and then uses exponential roulette to avoid being fully deterministic.

### What makes it return `false`
- No drone flights exist.
- The selected flight indices are invalid.
- No feasible truck insertion exists for any shortlisted flight.

### Key helper / code paths
- `src/operators/replace_drone_delivery.cpp`
- `rank_drone_demotion_candidates(...)`
- `replace_drone_delivery_with_bounds(...)`
- `shift_drone_indices_after_truck_insert(...)`

## Drone rendezvous shift best improvement

### Purpose / neighborhood
- Moves the launch and/or landing anchors of an existing drone flight while keeping the served customer on the same drone.
- This is the current mix's direct drone-schedule refinement move.

### How the candidate is generated
- `drone_rendezvous_shift_best_improvement` evaluates every existing drone flight.
- For each one, `find_best_shift_for_flight(...)` searches a local launch/land window:
  - launch window `3`
  - land window `3`
- It tries every `(new_launch, new_land)` pair within that bounded window, excluding the original pair.

### What happens if feasibility breaks
- A new anchor pair can break interval ordering, cause overlap with other flights on the same drone, or fail the full solution feasibility constraints.

### What repair path is used
- There is no repair fallback here.
- The operator uses strict feasibility filtering during search:
  - `can_place_without_overlap(...)` rejects interval overlaps before full evaluation.
  - The candidate flight list is sorted afterward with `sort_drone_collection(...)` when needed.
  - The full candidate must pass `master_check(...)`.

### What screening, bounding, or shortlist tricks it uses
- Search is local, not global: only a bounded anchor window is explored.
- Same-drone interval overlap is screened out before calling `master_check(...)`.
- The best candidate per flight is extracted first.
- Across flights, the operator keeps the top `5` candidates by objective and then samples from them with exponential roulette.

### What makes it return `false`
- No drone flights exist.
- The selected flight is invalid.
- No alternative anchor pair inside the bounded window is feasible.

### Key helper / code paths
- `src/operators/drone_rendezvous_shift.cpp`
- `find_best_shift_for_flight(...)`
- `get_intervals(...)`
- `overlaps(...)`
- `sort_drone_collection(...)`

## Shaw removal greedy repair (current Shaw-medium variant)

### Purpose / neighborhood
- Destroy-and-repair operator that removes a related set of served customers and reinserts them greedily.
- In the current mix, `src/main.cpp` uses `shaw_removal_greedy_repair_random_medium`, which sets `remove_count = 5`.
- For the shared repair terminology used below, see [current_solution_fixers.md](current_solution_fixers.md).

### How the candidate is generated
- `choose_shaw_removal_set(...)` first builds the candidate universe as all currently served customers, including both:
  - truck-served customers
  - drone-served customers
- It then selects a related subset to destroy.
- After destruction, `remove_customers_and_invalidated_flights(...)` produces a partial solution and a repair pool.
- The repair phase then rebuilds the missing customers greedily through truck or drone insertion.

### How relatedness-based removal works
- `approximate_position(...)` returns:
  - the truck index if the customer is currently on the truck route
  - otherwise the midpoint `(launch_idx + land_idx) / 2` if the customer is currently served by drone
- `shaw_relatedness(...)` is:
  - `min(truck_matrix[a][b], truck_matrix[b][a])`
  - plus `1000 * abs(approximate_position(a) - approximate_position(b))`
- Removal starts from one uniformly random seed customer from the full served-customer set.
- Each additional removal step works like this:
  1. choose one already-removed customer uniformly at random as the current reference
  2. sort all still-remaining customers by increasing relatedness to that reference
  3. sample a rank using `floor((U^3) * remaining_size)`
  4. remove that ranked customer
- Because the sampled rank uses `U^3`, the operator strongly favors very related customers, but it is not deterministic and can still occasionally reach farther away in the ranking.
- The loop continues until it reaches `remove_count` or runs out of remaining candidates.

### How the destroy phase rewrites the partial solution
- Truck customers in the removal set are removed from the truck route.
- The partial truck route is rebuilt with an `old_to_new` index map so surviving truck anchors can be remapped.
- Each existing drone flight is then examined.
- A drone flight is removed if any of these hold:
  - its delivered customer was explicitly removed
  - its launch anchor disappeared from the truck route
  - its non-terminal land anchor disappeared from the truck route
- So the destroy phase removes flights whose launch or land anchor disappears, not just flights whose delivered customer was explicitly selected for removal.
- When a flight is invalidated this way, its delivered customer is added to `repair_pool` exactly once through unique-insert logic.
- Surviving flights are preserved through the `old_to_new` remapping.

### What happens if feasibility breaks
- The destroy phase explicitly removes invalidated flights and expands the repair pool so the partial solution stays structurally consistent enough to rebuild.
- During repair, candidate insertions are evaluated only after canonicalizing terminal-depot landings and checking the full drone schedule consistency.

### What repair path is used
- There is no separate post-hoc fixer.
- The repair itself is a greedy feasibility-aware rebuild:
  - `greedy_repair(...)` randomizes the pool order first, but each round still picks the globally best next customer among all customers that can be inserted feasibly
  - `greedy_repair_one_customer(...)` tries every truck insertion position
  - it also tries every feasible direct drone launch/land pair on every drone
- Candidate screening and validation are built into that repair:
  - `pure_drone_flight_within_limit(...)` screens raw drone insertions,
  - `evaluate_candidate_with_timing(...)` canonicalizes depot landings,
  - `canonical_drone_schedule_consistent(...)` rejects overlapping or ill-ordered schedules,
  - route timing is recomputed and each drone flight is checked against the time limit with actual waiting included.
- Only a fully feasible repaired solution is accepted.

### How greedy repair works
- `greedy_repair_one_customer(...)` evaluates two move families for one customer:
  - insert onto the truck at every valid position
  - insert as a direct drone flight on any drone, for every launch/land pair that passes the pure-drone precheck
- Truck insertion immediately shifts downstream drone indices before full candidate evaluation.
- Each candidate is then validated by `evaluate_candidate_with_timing(...)`.
- `greedy_repair(...)` repeats this at the pool level:
  - try inserting every remaining customer into a copy of the current partial solution
  - keep the best feasible repaired result across those customers
  - commit only that best repaired result
  - remove the chosen customer from the pool
- This makes the repair greedy globally across customers, even though the pool order was randomized first.

### What screening, bounding, or shortlist tricks it uses
- Relatedness combines geometry and route position.
- Candidate removal is Shaw-biased rather than uniform.
- Pure-drone feasibility cache is used before expensive route-timing evaluation.
- Repair is greedy best-insertion rather than exhaustive over the full removed set at once.

### What makes it return `false`
- Removal size is non-positive.
- Route too short.
- No removal set could be built.
- Repair pool ends up empty.
- No feasible greedy repair exists.
- Final repaired solution is rejected unless it is both:
  - feasible
  - strictly improving over the original objective

### Key helper / code paths
- `src/operators/shaw_removal_greedy_repair.cpp`
- `choose_shaw_removal_set(...)`
- `remove_customers_and_invalidated_flights(...)`
- `greedy_repair_one_customer(...)`
- `greedy_repair(...)`
- `evaluate_candidate_with_timing(...)`

## Notes

- The current mix in `src/main.cpp` uses these initial weights:
  - `NN-Reassign`: `0.40`
  - `Two-Opt Arrival Screened`: `0.50`
  - `Truck replacement greedy`: `0.55`
  - `Targeted drone-to-truck`: `0.15`
  - `Drone rendezvous shift best improvement`: `0.90`
  - `Shaw removal greedy repair`: `0.80`
- These are only starting weights. GAM updates them online during the run.
