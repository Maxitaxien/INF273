cmake_minimum_required(VERSION 3.16)

project(truck_and_drone LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQIORED ON)

add_executable(truck_and_drone
    src/main.cpp
    src/datahandling/reader.cpp
    src/datahandling/convert_to_submission.cpp
    src/datahandling/save_to_csv.cpp
    src/algorithms/simple_initial_solution.cpp
    src/algorithms/nearest_neighbour.cpp
    src/algorithms/greedy_drone_cover.cpp
    src/algorithms/random_valid_solution.cpp
    src/algorithms/blind_random_search.cpp
    src/verification/feasibility_check.cpp
    src/verification/objective_value.cpp
)

target_include_directories(truck_and_drone
    PUBLIC include
)