#include "operators/or_opt_segment_relocate.h"
#include "operators/customer_slot_helpers.h"
#include "operators/drone_planner.h"
#include "solution_fixers/solution_fixers.h"
#include "general/random.h"
#include "datahandling/instance.h"
#include "verification/solution.h"
#include "verification/feasibility_check.h"
#include "verification/objective_value.h"
#include <algorithm>
#include <numeric>
#include <utility>
#include <vector>

namespace
{
bool repair_with_drone_planner_if_needed(
    const Instance &inst,
    Solution &candidate)
{
    if (master_check(inst, candidate, false))
    {
        return true;
    }

    const int iterations = inst.n >= 50 ? 1 : 2;
    const int max_flights_per_customer = inst.n >= 50 ? 8 : 12;
    const auto [planned_cost, planned_solution] = drone_planner(
        inst,
        candidate,
        iterations,
        max_flights_per_customer);
    (void)planned_cost;
    if (!master_check(inst, planned_solution, false))
    {
        return false;
    }

    candidate = planned_solution;
    return true;
}

bool relocate_customer_block(
    std::vector<int> &values,
    int start_idx,
    int segment_length,
    int insert_after_idx)
{
    const int slot_count = (int)(values.size());
    if (segment_length <= 0 || start_idx < 0 || start_idx + segment_length > slot_count)
    {
        return false;
    }

    if (insert_after_idx < 0 || insert_after_idx >= slot_count)
    {
        return false;
    }

    if (insert_after_idx >= start_idx - 1 &&
        insert_after_idx < start_idx + segment_length)
    {
        return false;
    }

    const std::vector<int> moved_block(
        values.begin() + start_idx,
        values.begin() + start_idx + segment_length);
    values.erase(
        values.begin() + start_idx,
        values.begin() + start_idx + segment_length);

    int insert_pos = insert_after_idx + 1;
    if (insert_after_idx > start_idx)
    {
        insert_pos -= segment_length;
    }

    values.insert(values.begin() + insert_pos, moved_block.begin(), moved_block.end());
    return true;
}

std::vector<int> valid_insert_after_positions(int slot_count, int start_idx, int segment_length)
{
    std::vector<int> positions;
    positions.reserve(slot_count);
    for (int insert_after_idx = 0; insert_after_idx < slot_count; ++insert_after_idx)
    {
        if (insert_after_idx >= start_idx - 1 &&
            insert_after_idx < start_idx + segment_length)
        {
            continue;
        }

        positions.push_back(insert_after_idx);
    }

    return positions;
}

std::vector<int> collect_customer_values(
    const Solution &sol,
    const std::vector<CustomerSlot> &slots)
{
    std::vector<int> values;
    values.reserve(slots.size());
    for (const CustomerSlot &slot : slots)
    {
        values.push_back(read_customer_at_slot(sol, slot));
    }
    return values;
}

bool apply_or_opt_candidate(
    const Instance &inst,
    const Solution &base_solution,
    const std::vector<CustomerSlot> &slots,
    const std::vector<int> &base_values,
    Solution &candidate,
    std::vector<int> &working_values,
    int start_idx,
    int segment_length,
    int insert_after_idx)
{
    working_values = base_values;
    if (!relocate_customer_block(working_values, start_idx, segment_length, insert_after_idx))
    {
        return false;
    }

    candidate = base_solution;
    for (int idx = 0; idx < (int)(slots.size()); ++idx)
    {
        write_customer_at_slot(candidate, slots[idx], working_values[idx]);
    }

    remap_drone_anchor_indices_by_node(base_solution, candidate);
    return repair_with_drone_planner_if_needed(inst, candidate);
}
}

bool or_opt_segment_relocate(
    const Instance &inst,
    Solution &sol,
    int start_idx,
    int segment_length,
    int insert_after_idx)
{
    const std::vector<CustomerSlot> slots = collect_customer_slots(sol);
    const int slot_count = (int)(slots.size());
    if (slot_count == 0)
    {
        return false;
    }

    const std::vector<int> base_values = collect_customer_values(sol, slots);
    std::vector<int> working_values;
    Solution candidate = sol;
    if (!apply_or_opt_candidate(
            inst,
            sol,
            slots,
            base_values,
            candidate,
            working_values,
            start_idx,
            segment_length,
            insert_after_idx))
    {
        return false;
    }

    sol = std::move(candidate);
    return true;
}

bool or_opt_segment_relocate_first_improvement(
    const Instance &inst,
    Solution &sol)
{
    const std::vector<CustomerSlot> slots = collect_customer_slots(sol);
    const int slot_count = (int)(slots.size());
    if (slot_count < 2)
    {
        return false;
    }

    const long long current_cost = objective_function_impl(inst, sol);
    const int max_segment_length = std::min(3, slot_count);
    const std::vector<int> base_values = collect_customer_values(sol, slots);
    std::vector<int> working_values;
    Solution candidate = sol;

    for (int segment_length = 1; segment_length <= max_segment_length; ++segment_length)
    {
        for (int start_idx = 0; start_idx + segment_length <= slot_count; ++start_idx)
        {
            const std::vector<int> insert_positions =
                valid_insert_after_positions(slot_count, start_idx, segment_length);
            for (int insert_after_idx : insert_positions)
            {
                if (!apply_or_opt_candidate(
                        inst,
                        sol,
                        slots,
                        base_values,
                        candidate,
                        working_values,
                        start_idx,
                        segment_length,
                        insert_after_idx))
                {
                    continue;
                }

                if (objective_function_impl(inst, candidate) < current_cost)
                {
                    sol = std::move(candidate);
                    return true;
                }
            }
        }
    }

    return false;
}

bool or_opt_segment_relocate_random(
    const Instance &inst,
    Solution &sol)
{
    const std::vector<CustomerSlot> slots = collect_customer_slots(sol);
    const int slot_count = (int)(slots.size());
    if (slot_count < 2)
    {
        return false;
    }

    const int segment_length = rand_int(1, std::min(2, slot_count));
    const std::vector<int> selected_indices =
        sample_contiguous_slot_indices(slot_count, segment_length);
    if (selected_indices.empty())
    {
        return false;
    }

    const int start_idx = selected_indices.front();
    std::vector<int> insert_positions =
        valid_insert_after_positions(slot_count, start_idx, segment_length);
    if (insert_positions.empty())
    {
        return false;
    }

    std::shuffle(insert_positions.begin(), insert_positions.end(), gen);
    const int max_attempts = std::min(
        (int)(insert_positions.size()),
        inst.n >= 50 ? 4 : 8);
    const std::vector<int> base_values = collect_customer_values(sol, slots);
    std::vector<int> working_values;
    Solution candidate = sol;

    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
        const int insert_after_idx = insert_positions[attempt];
        if (!apply_or_opt_candidate(
                inst,
                sol,
                slots,
                base_values,
                candidate,
                working_values,
                start_idx,
                segment_length,
                insert_after_idx))
        {
            continue;
        }

        sol = std::move(candidate);
        return true;
    }

    return false;
}
