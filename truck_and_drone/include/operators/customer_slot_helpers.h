#pragma once

#include "verification/solution.h"
#include <vector>

/**
 * Helpers for collecting truck and drone deliveries into a single data strucutre,
 * CustomerSlot. 
 */


/**
 * Tracks whether the customer belongs to the truck or drone and where it is.
 */
struct CustomerSlot
{
    bool on_truck = false;
    int index = -1;
    int drone = -1;
};

/**
 * Assemble a vector of customer slots.
 */
std::vector<CustomerSlot> collect_customer_slots(const Solution &sol);

/**
 * Checks relevant solution part and reads customer based on index
 */
int read_customer_at_slot(const Solution &sol, const CustomerSlot &slot);

/**
 * Modify solution based on slot
 */
void write_customer_at_slot(Solution &sol, const CustomerSlot &slot, int customer);

/**
 * Random sample of available slot indices of size sample_size
 */
std::vector<int> sample_slot_indices(int slot_count, int sample_size);

/**
 * Random contiguous block of slot indices of size sample_size.
 *
 * The returned indices are consecutive in the combined customer-slot vector.
 */
std::vector<int> sample_contiguous_slot_indices(int slot_count, int sample_size);

/**
 * Returns all contiguous blocks of slot indices.
 */
std::vector<std::vector<int>> get_all_contiguous_indices(int slot_count, int sample_size);
