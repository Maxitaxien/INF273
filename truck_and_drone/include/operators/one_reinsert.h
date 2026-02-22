#pragma once
#include "verification/solution.h"
#include "datahandling/instance.h"

/**
 * 
 * @param pop 1, 2 or 3- which list to pop from
 * @param insert 1, 2 or 3 - which list to re-insert into
 * @param idx the index to reinsert into
 * 
 * Afterwards, fixes drone flights to ensure consistency.
 */
bool one_reinsert(const Instance& instance, Solution& sol, int pop, int insert, int idx);