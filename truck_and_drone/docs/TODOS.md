# TODOS

## Experiments from Ahmad's discord post:

Probably easiest is if the run returns this information, then we just read the data from the csvs and generate plots

- Temp:
  - Implement SA temp + acceptance in GAM. Also based on "badness" of solution
  - Store temp values
  - Make plots for smallest and largest instances: x: iter, y: temp

- P(accept)
  - plot prob of acceptance where objective value worsens (delta > 0)
  - scatterplot: X: iter, y: p(accept)
  - plots for smallest and largest instances

- Best solution iteration
  - save iteration where best solution is found
  - include this in .csv file

- delta val per operator
  - plot change in objective value for each operator
  - scatterr plot: x: iter, y: delta val
  - separate plots for each operator
  - plot for run where best solution was achieved
    - so also store this information in runner where best sol is achieved

- operator weights
  - plot weights of operators across different segments, using different colours
  - x: segment, y: weights (lineplot)
  - plots for all instances in run where best solution was achieved

- report objective value (already impl)

## Other

- Solution visualizer

~~- Insertion operators~~

~~- Removal operators~~

Split greedy insert into swapping around in truck route, and inserting into drone?
May make it easier to learn separate weights in ALNS.
Also, evaluating all greedy insert positions is just too slow.

- Truck and Drone paper
  ~~- Drone planner heuristic~~

IMPORTANT: Create a more powerful drone -> truck neighbourhood operator.

Plan:

Add a combined-vector Or-opt operator.
This is the best easy quality upgrade. Move a block of 1-3 customers inside the combined Part 1 + Part 2 sequence, then repair with the drone planner if needed. It is stronger than random swaps, still simple to code, and matches the order-first/repair-second pattern you already use.

Precompute and cache cheap drone feasibility filters.
Build once per instance:

which customers are drone-eligible at all
candidate launch/land pairs or at least bounded launch/land windows
Then use those filters inside drone_planner, replace_truck_delivery, and random repair operators. This is the highest leverage speed improvement without changing the 10k-iteration rule.
Make Three-Opt scheduled rather than always sampled.
Keep it, but call it rarely or only after stagnation/segment boundaries. It is a good intensifier, not a good default workhorse. That should preserve its quality contribution while cutting a lot of wasted time on larger instances.

Remove or heavily downweight Drone replacement greedy.
Based on our profiling, it is the weakest operator in the current setup. If you want to keep it, use it only as a shake operator after stagnation, not in the core roulette mix.

Add planner trigger rules instead of sampling planner as a regular move.
Use drone_planner_improve only:

after a sequence-changing move causes infeasibility
after accepted Or-opt/2-opt/3-opt moves every N iterations
during stagnation escape
That is more consistent with the literature than giving the planner its own frequent selection weight.
Recommended order

Or-opt
cached drone feasibility filters
remove/downweight Drone replacement greedy
schedule Three-Opt
planner trigger rules
