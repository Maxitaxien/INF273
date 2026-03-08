cmake_minimum_required(VERSION 3.16)
project(truck_and_drone LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Core library
add_library(core
    src/datahandling/reader.cpp
    src/datahandling/convert_to_submission.cpp
    src/datahandling/save_to_csv.cpp
    src/datahandling/file_helpers.cpp
    src/datahandling/create_markdown_tables.cpp
    src/algorithms/simple_initial_solution.cpp
    src/algorithms/nearest_neighbour.cpp
    src/algorithms/greedy_drone_cover.cpp
    src/algorithms/random_valid_solution.cpp
    src/algorithms/blind_random_search.cpp
    src/algorithms/local_search.cpp
    src/algorithms/simulated_annealing.cpp
    src/general/get_truck_arrival_times.cpp
    src/general/split.cpp
    src/general/sort_drone_collection.cpp
    src/operators/helpers.cpp
    src/operators/interval_helpers.cpp
    src/operators/operator.cpp
    src/operators/one_reinsert.cpp
    src/operators/replace_truck_delivery.cpp
    src/operators/two_opt.cpp
    src/operators/nearest_neighbour_reassign.cpp
    src/operators/solution_fixers.cpp
    src/verification/feasibility_check.cpp
    src/verification/objective_value.cpp
    src/runners/run_algorithm.cpp
    src/runners/wrappers.cpp
)

target_include_directories(core PUBLIC include)

# Main program
add_executable(truck_and_drone src/main.cpp)
target_link_libraries(truck_and_drone PRIVATE core)

# Tests
add_executable(test_truck_and_drone
    src/tests/test_main.cpp
)

target_link_libraries(test_truck_and_drone PRIVATE core)