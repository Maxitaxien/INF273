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

/**
 * Alternative version of inclues_all_nodes - splits string to verify.
 * (Note that other feasibility checks are not setup to check strings)
 */
bool includes_all_nodes(int n, const std::string& submission, bool debug);

std::vector<long long> get_truck_arrival_times_at_node(
    const Instance& instance,
    const Solution& solution,
    std::vector<long long>& drone_available,
    long long& total_drone_arrival      // new output parameter
);

bool specific_drone_flight_under_lim(const Instance& instance,
                                     const Solution& solution,
                                     int drone_idx,      
                                     int flight_idx);

/**
 * Check whether all drone flights are all under the flight limit.
 * (Should mainly be used for debugging. For feasibility checks, prefer all_drone_flights_feasible,
 * which performs the role of both all_drone_flights_under_lim and drone_flights_consistent all in one)
 */
bool all_drone_flights_under_lim(const Instance& problem_instance, const Solution& solution, bool debug);

/**
 * Check whether all drones launch before they land.
 * (Should mainly be used for debugging. For feasibility checks, prefer all_drone_flights_feasible,
 * which performs the role of both all_drone_flights_under_lim and drone_flights_consistent all in one)
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