# Current GAM Setup

This is the authoritative reference for the current General Adaptive Metaheuristic (GAM) setup in this repository.

## Entry points and experiment definition

- The GAM runner entry points live in `include/runners/run_algorithm.h` and `src/runners/run_algorithm.cpp`.
- The current experiment mix is defined in `src/main.cpp`.
- `main()` currently runs two experiments over:
  - `F_10`, `F_20`, `F_50`, `F_100`
  - `R_10`, `R_20`, `R_50`, `R_100`
- Each experiment runs `5` independent runs per dataset.
- The two current experiment labels are:
  - `Current mix - keep cache after escape`
  - `Current mix - clear cache after escape`

## Current mix and weights

- The current mix has six operators:
  - `NN-Reassign`
  - `Two-Opt Arrival Screened`
  - `Truck replacement greedy`
  - `Targeted drone-to-truck`
  - `Drone rendezvous shift best improvement`
  - `Shaw removal greedy repair`
- The starting weights from `src/main.cpp` are:
  - `0.40`
  - `0.50`
  - `0.55`
  - `0.15`
  - `0.90`
  - `0.80`
- These are only initial weights. Selection remains roulette-wheel based, but the weights are updated online by GAM.

## Initial solution behavior

- `general_adaptive_metaheuristic(...)` currently ignores the passed-in initial solution and replaces it with `nearest_neighbour(instance)` at the start of the run.
- That means the runner's `simple_initial_solution(...)` is still used for baseline reporting outside GAM, but the GAM search itself always starts from nearest neighbour right now.

## Time budgets and timed mode

- The dataset-specific default GAM time budgets are defined in `src/runners/run_algorithm.cpp`:
  - `n = 10`: `5` seconds
  - `n = 20`: `20` seconds
  - `n = 50`: `60` seconds
  - `n = 100`: `510` seconds
- Because `run_gam_experiments(...)` resolves these budgets per instance, the current main-program GAM runs are in timed mode.
- If `time_limit_s <= 0`, GAM switches to untimed mode instead.

## Core loop shape

- Timed mode:
  - runs until wall-clock time reaches the chosen budget
  - weight updates are spread across about `50` time segments
- Untimed mode:
  - runs for `10000` iterations
  - updates weights every `100` iterations
- In both modes, operators are selected by roulette wheel from the active weight vector.

## Acceptance modes

`GAMConfig.acceptance_mode` supports two modes.

### SimulatedAnnealing

- This is the current default.
- Improving or equal moves are accepted directly.
- Worsening moves are accepted with probability:
  - `exp(-delta / temperature)`

### BestRelativeRRT

- Worsening moves are accepted if:
  - `candidate_cost <= best_cost + allowed_deviation`
- `allowed_deviation` is:
  - `allowed_deviation_fraction * (1 - progress) * best_cost`
- So the allowed slack shrinks over the run as progress approaches `1.0`.

## Initial temperature sampling

- The initial SA temperature is not fixed by hand.
- GAM samples positive deltas from operator-generated neighbors using `average_positive_delta_sample(...)`.
- Sampling details:
  - target positive samples: `50`
  - max attempts: `250`
  - if there are no positive deltas, it falls back to `max(1.0, abs(reference_cost) * 0.01)`
- The target initial worsening acceptance probability is:
  - `0.18` when `n < 50`
  - `0.12` when `n >= 50`
- Temperature is then computed as:
  - `-sampled_positive_delta / log(target_probability)`

## Cooling behavior

### Untimed mode

- Untimed SA cools geometrically from the initial temperature down toward:
  - `phase_final_temperature = max(1.0, phase_initial_temperature * 0.05)`
- The cooling rate is chosen so that this decay spans the full iteration budget.

### Timed mode

- Timed SA also aims for `phase_final_temperature = max(1.0, phase_initial_temperature * 0.05)`.
- Instead of decrementing once per iteration, it cools according to elapsed phase time.
- That means slower or faster operators still track the intended time-based schedule.

## Phase transition and reset behavior

- `GAMConfig.phase_one_fraction` splits the run into phase one and phase two by time.
- `phase_one_operator_names` and `phase_two_operator_names` can restrict which operators are active in each phase.
- If a phase name list is empty, GAM falls back to all operators.
- If a phase name list is provided but nothing resolves, GAM also falls back to all operators.
- `reset_acceptance_each_phase` controls whether the acceptance schedule resets when the run moves from phase one to phase two.
- When the reset is enabled and a real phase transition happens:
  - the phase clock is reset
  - SA temperature is resampled from the current incumbent and active phase operator pool
  - non-improving iteration count is reset

### Current practical note

- The current experiments use the default `phase_one_fraction = 1.0`.
- That means the whole timed run stays in phase one unless the config is changed, so phase-two restrictions and the phase reset are currently dormant.

## Reheating after escape

- When the stagnation threshold triggers an escape and acceptance mode is SA, GAM reheats with:
  - `temperature = max(temperature, phase_initial_temperature * 0.25)`
- This is a partial reheating, not a full reset to the original phase temperature.

## Escape mechanism

- GAM tracks `non_improving_iterations`.
- When that counter reaches `400`, it triggers an escape.
- The escape budget is `20` iterations or escape steps.

### ExchangeKLarge

- This is the current default `GAMEscapeMode`.
- It repeatedly applies `exchange_k_large(...)` to the incumbent.
- Each feasible accepted escape step replaces the incumbent.
- It also tracks the best solution seen during the escape, even though the returned incumbent is the last accepted endpoint.

### LegacyGAM

- This runs a short weighted random walk over the current operator pool.
- Selection still uses roulette-wheel weights.
- Like `ExchangeKLarge`, it returns the final incumbent while also reporting the best solution seen during the escape.

## Cache behavior

- GAM uses `GAMSolutionCache`, keyed by the serialized canonical solution.
- Solutions are canonicalized through `canonicalize_terminal_depot_landings(...)` before caching and evaluation.
- The cache stores:
  - whether feasibility is known
  - the feasibility verdict
  - whether the objective is known
  - the cached objective

### Cache-clearing option

- `GAMConfig.clear_solution_cache_after_escape` controls whether the entire cache is cleared immediately after each escape.
- The current main program runs both variants:
  - one keeping the cache after escapes
  - one clearing it after escapes

## Feasibility mode

`GAMConfig.feasibility_mode` supports two modes.

### AssumeFeasible

- `evaluate_solution_with_cache(...)` skips `master_check(...)`.
- The solution is treated as feasible and the objective is cached directly.
- This is the current experiment setting in `src/main.cpp`.
- It relies on the current operator mix and escape path already producing valid candidates before GAM evaluates them.

### VerifyWithMasterCheck

- `evaluate_solution_with_cache(...)` runs `master_check(...)` before caching feasibility and objective.
- This is slower, but it is the safer debugging mode for new or suspect operators.

## How operator weights are affected

Weight updates happen segment-by-segment, not after every move.

### Per-move reward signal

- `compute_operator_reward(...)` adds:
  - normalized improvement when `delta < 0`
  - `+0.10` if the move was accepted
  - `+2.0` if it produced a new global best
  - `+0.05` if it produced a cache-new solution
- Normalized improvement is capped at `5.0` and scaled by the rolling `reward_delta_scale`.

### Additional segment statistics

- Each operator also tracks within-segment:
  - uses
  - successful feasible candidates
  - accepted moves
  - improving accepted moves
  - new bests
  - cumulative reward

### Segment update constants

- `reaction_factor = 0.30`
- `min_weight_ratio = 0.20`
- `max_weight_ratio = 3.50`
- `accepted_bonus = 0.15`
- `success_bonus = 0.10`
- `improvement_bonus = 0.25`
- `best_bonus = 0.40`
- `pull_to_prior = 0.05`

### Segment update shape

- Performance per used operator is:
  - `score_per_use`
  - plus `0.35 * improvement_per_use`
  - plus the accepted/success/improvement/best rate bonuses
- That signal is centered by the mean performance of active operators in the segment.
- The centered signal is exponentiated and applied multiplicatively to the current weight.
- Then the weight is softly pulled back toward its initial weight.
- Finally weights are clipped between:
  - `0.20 * initial_weight`
  - `3.50 * initial_weight`
- After clipping, the whole vector is rescaled so the total weight matches the original total initial weight.

## Configuration options

`GAMConfig` currently exposes:

- `phase_one_fraction`
  - fraction of the timed run assigned to phase one
- `allowed_deviation_fraction`
  - the slack factor used by `BestRelativeRRT`
- `reset_acceptance_each_phase`
  - whether to rebuild the acceptance schedule when phase two starts
- `clear_solution_cache_after_escape`
  - whether to drop the full GAM solution cache after each escape
- `acceptance_mode`
  - `SimulatedAnnealing` or `BestRelativeRRT`
- `escape_mode`
  - `ExchangeKLarge` or `LegacyGAM`
- `feasibility_mode`
  - `AssumeFeasible` or `VerifyWithMasterCheck`
- `phase_one_operator_names`
  - optional exact-name filter for the phase-one operator pool
- `phase_two_operator_names`
  - optional exact-name filter for the phase-two operator pool

## Persisted statistics files

For the best run on each dataset, `save_gam_statistics(...)` writes a dataset statistics directory containing:

### `summary.csv`

- One row summarizing the saved best run for that dataset.
- Fields:
  - `best_run`
  - `best_found_iteration`
  - `operator_failures`
  - `infeasible_candidates`
  - `accepted_moves`
  - `improving_accepts`
  - `non_improving_accepts`
  - `best_updates`

### `trace.csv`

- Per-iteration trace for the saved best run.
- Includes:
  - iteration index
  - operator id and operator name
  - delta
  - whether delta was known
  - incumbent objective
  - best objective
  - temperature
  - allowed deviation
  - virtual schedule fraction
  - worsening acceptance probability
  - operator runtime in milliseconds

### `weights.csv`

- Per-segment operator weights for the saved best run.
- Includes:
  - segment index
  - iteration when the segment update was recorded
  - operator id and name
  - weight

### `runs.csv`

- One row per run for that dataset.
- Includes:
  - run number
  - final objective
  - best-found iteration

### `operators.csv`

- Aggregate operator statistics for the saved best run.
- Includes:
  - uses
  - changed candidates
  - failures
  - infeasible candidates
  - feasible candidates
  - accepted moves
  - improving accepts
  - new bests
  - delta sample counts and sums
  - total runtime
  - average runtime

## Plotting pipeline note

- `scripts/plot_run_statistics.py` currently consumes:
  - `trace.csv`
  - `weights.csv`
  - `runs.csv`
  - `operators.csv`
- It does not depend on the trimmed `summary.csv` schema for the GAM visualizations.
