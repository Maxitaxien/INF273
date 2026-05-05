# README

## Building and running

This project includes all dependencies natively. Configure, build, and run the
project from `build-native/`, since the dataset paths used by the binary are
relative to that working directory.

```bash
cmake -S . -B build-native -DCMAKE_BUILD_TYPE=Release
cmake --build build-native -j"$(nproc)"
cd build-native
./truck_and_drone
```

The checked-in `src/main.cpp` is configured to run a one-run parallel GAM batch
over `F10`, `R10`, `F20`, `R50`, and `F100` using the current operator mix,
current tuned weights, and the parallel runner's default shared batch budget.
`main.cpp` also contains a commented full-benchmark example for the full
benchmark dataset suite with 10 sequential iterations.

## data/

Contains datasets for the project. Paths to these datasets are registered in `include/datahandling/datasets.h`.

## docs/

This directory contains some documentation for the current and best-performing mix of operators, as well as documentation for the in-use solution fixers and final algorithm version (`algorithms/gam.cpp`)

Also in this directory are some reference python objective function and feasibility checkers from: https://github.com/Ahmad-Hemmati/Truck-and-Drones-Contest

## src/algorithms/

Contains the main logic for a few construction algorithms (`simple_initial_solution.cpp`, `nearest_neighbour.cpp`, `greedy_drone_cover.cpp`) as well as a random solution generator + searcher (`random_valid_solution.cpp` + `blind_random_search.cpp`) and the main metaheuristic algorithms (`local_search.cpp`, `simulated_annealing.cpp`, `gam.cpp`). Note that

As `gam.cpp` is the main final algorithm, it has more significantly more details than the other algorithms, such as a dedicated escape algorithm file and solution caching file (`gam_escape_algorithm.cpp`, `gam_solution_cache.cpp`). See `docs/current_gam_setup.md` for more details.

## src/datahandling/

Contains methods for reading in data, converting to the submission string format and generating the result tables under `runs/`.

## src/general/

Contains some general helpers such as getting positions of customers on a truck route, some utility methods and random method implementations.

## src/operators/

Contains operators used in the project as well as a few helper methods, where most are used in the final mix.

## src/runners/

Contain helper methods for running algorithm experiments. These are the endpoints which should be called from `main.cpp` to initiate a run. The runners have support for running multiple different algorithms at once or running ablation experiments on a single algorithm. There is also a method for running instances in parallell, which only supports a single run per dataset in this mode.

## src/solution_fixers/

Contains a range of different solution fixers applied after operators destroy
the solution feasibility. This usually involves reassignment of drone launch
and land spots.

See `docs/current_solution_fixers.md` for more details.

## src/tsp/

Contains a C API (`concorde_linkern_c_api.c`) to work with the Concorde Linkern solver and a solver class to simplify using the solver (`linkernsolver.cpp`).

_The files within src/tsp/ are created mainly with LLM usage._

## src/verification/

Contains feasibility checkers and objective value calculation, as well as some simple helpers and methods for visualizing the generated solutions.

_The visualization logic within src/verification/ is created mainly with LLM usage._

## third_party/

Contains third party Concorde solver and unused LKH solver used for internal TSP solving in Exchange K operators.
