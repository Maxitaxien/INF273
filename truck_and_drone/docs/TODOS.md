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
