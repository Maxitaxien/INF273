#pragma once

#include <string>
#include "datahandling/instance.h"
#include "verification/solution.h"

/**
 * Helper function to split a string, used for feasibility checker string versions
 */
std::vector<std::string> split(const std::string& s, char delimiter);

/**
 * Preferred version of includes_all_nodes - works more efficiently on a solution
 */
bool includes_all_nodes(int n, const Solution& solution, bool debug);

bool specific_drone_flight_under_lim(const Instance& instance,
                                     const Solution& solution,
                                     int drone_idx,
                                     int flight_idx);

/**
 * Check whether all drone flights are under the flight limit including truck waiting at the rendezvous.
 */
bool all_drone_flights_under_lim_with_wait(const Instance& problem_instance, const Solution& solution, bool debug);

/**
 * Check whether all drones launch before they land.
 */
bool drone_flights_consistent(const Solution& solution, bool debug);

/**
 * Checks flight limit and consistency of the drones.
 */
bool all_drone_flights_feasible(const Instance& problem_instance, const Solution& solution, bool debug);


/**
 * Performs an includes_all_nodes_check and all_drone_flights_feasible check to give a final feasibility and validity verdict.
 * In all other cases than debugging specific issues, it will be sufficient to run this function for feasibility
 * and with debug = false.
 * 
 * Note that we rely on the rules of the algorithm to make sure other minor validity details are correct,
 * such as starting and ending at the depot.
 */
bool master_check(const Instance& problem_instance, const Solution& solution, bool debug);
