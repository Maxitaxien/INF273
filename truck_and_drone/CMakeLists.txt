cmake_minimum_required(VERSION 3.16)
project(truck_and_drone LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_program(CONCORDE_MAKE_EXECUTABLE NAMES make gmake REQUIRED)


# execute_process(
#     COMMAND ${Python3_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
#     OUTPUT_VARIABLE Python3_NumPy_INCLUDE_DIR
#     OUTPUT_STRIP_TRAILING_WHITESPACE
# )

set(CONCORDE_SOURCE_DIR "${CMAKE_SOURCE_DIR}/third_party/concorde")
set(CONCORDE_BUILD_DIR "${CMAKE_BINARY_DIR}/third_party/concorde")
set(CONCORDE_MAKEFILE "${CONCORDE_BUILD_DIR}/Makefile")
set(CONCORDE_CONFIG_HEADER "${CONCORDE_BUILD_DIR}/INCLUDE/config.h")
set(CONCORDE_LIBRARY "${CONCORDE_BUILD_DIR}/concorde.a")
file(MAKE_DIRECTORY "${CONCORDE_BUILD_DIR}")

file(GLOB_RECURSE CONCORDE_BUILD_DEPS CONFIGURE_DEPENDS
    "${CONCORDE_SOURCE_DIR}/*.c"
    "${CONCORDE_SOURCE_DIR}/*.h"
    "${CONCORDE_SOURCE_DIR}/*.in"
)
list(APPEND CONCORDE_BUILD_DEPS
    "${CONCORDE_SOURCE_DIR}/configure"
    "${CONCORDE_SOURCE_DIR}/configure.in"
)

add_custom_command(
    OUTPUT
        "${CONCORDE_MAKEFILE}"
        "${CONCORDE_CONFIG_HEADER}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CONCORDE_BUILD_DIR}"
    COMMAND "${CONCORDE_SOURCE_DIR}/configure"
    WORKING_DIRECTORY "${CONCORDE_BUILD_DIR}"
    DEPENDS
        "${CONCORDE_SOURCE_DIR}/configure"
        "${CONCORDE_SOURCE_DIR}/configure.in"
        "${CONCORDE_SOURCE_DIR}/Makefile.in"
        "${CONCORDE_SOURCE_DIR}/INCLUDE/config.h.in"
        "${CONCORDE_SOURCE_DIR}/INCLUDE/Makefile.common.in"
    VERBATIM
)

add_custom_command(
    OUTPUT "${CONCORDE_LIBRARY}"
    COMMAND "${CONCORDE_MAKE_EXECUTABLE}" concorde.a
    WORKING_DIRECTORY "${CONCORDE_BUILD_DIR}"
    DEPENDS
        ${CONCORDE_BUILD_DEPS}
        "${CONCORDE_MAKEFILE}"
        "${CONCORDE_CONFIG_HEADER}"
    VERBATIM
)

add_custom_target(concorde_build
    DEPENDS "${CONCORDE_LIBRARY}"
)

# Core library
add_library(core
    src/datahandling/reader.cpp
    src/datahandling/instance_preprocessing.cpp
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
    src/algorithms/gam.cpp
    src/algorithms/gam_solution_cache.cpp
    src/algorithms/gam_escape_algorithm.cpp
    src/general/get_customer_positions.cpp
    src/general/get_not_covered_by_truck.cpp
    src/general/get_truck_arrival_times.cpp
    src/general/split.cpp
    src/general/sort_drone_collection.cpp
    src/general/random.cpp
    src/general/roulette_wheel_selection.cpp
    src/operators/helpers.cpp
    src/operators/customer_slot_helpers.cpp
    src/operators/interval_helpers.cpp
    src/operators/route_timing.cpp
    src/operators/operator.cpp
    src/operators/exchange_k_v2.cpp
    src/operators/concorde_tsp.cpp
    src/operators/concorde_k_swap_rebuild.cpp
    src/operators/exact_segment_reopt.cpp
    src/operators/alns/random_removal.cpp
    src/operators/alns/greedy_insert.cpp
    src/operators/alns/alns_composite.cpp
    src/operators/one_reinsert.cpp
    src/operators/replace_drone_delivery.cpp
    src/operators/replace_truck_delivery.cpp
    src/operators/two_opt.cpp
    src/operators/three_opt.cpp
    src/operators/or_opt_segment_relocate.cpp
    src/operators/origin_destination_relocation.cpp
    src/operators/drone_rendezvous_shift.cpp
    src/operators/late_customer_drone_promotion.cpp
    src/operators/nearest_neighbour_reassign.cpp
    src/solution_fixers/helper_methods.cpp
    src/solution_fixers/launch_land_assignment.cpp
    src/solution_fixers/two_opt_fixers.cpp
    src/solution_fixers/simple_feasibility_fixers.cpp
    src/solution_fixers/drone_planner_fixers.cpp
    src/operators/drone_planner.cpp
    src/verification/feasibility_check.cpp
    src/verification/objective_value.cpp
    src/verification/solution_utils.cpp
    src/verification/solution_visualization.cpp
    src/runners/run_algorithm.cpp
    src/runners/wrappers.cpp

    src/tsp/concorde_linkern_c_api.c
    src/tsp/linkernsolver.cpp
)

target_include_directories(core PUBLIC
    include
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIR}
)

target_include_directories(core PRIVATE
    ${CONCORDE_BUILD_DIR}/INCLUDE
    ${CONCORDE_SOURCE_DIR}/INCLUDE
)

target_link_libraries(core PUBLIC
    ${Python3_LIBRARIES}
    m
)

target_link_libraries(core PRIVATE
    ${CONCORDE_LIBRARY}
)

add_dependencies(core concorde_build)

target_include_directories(core PUBLIC ${CMAKE_SOURCE_DIR}/external)

# Main program
add_executable(truck_and_drone src/main.cpp)
target_link_libraries(truck_and_drone PRIVATE core)

# Tests
add_executable(test_truck_and_drone
    src/tests/test_main.cpp
)

target_link_libraries(test_truck_and_drone PRIVATE core)
