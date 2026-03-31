#include "operators/customer_slot_helpers.h"
#include "verification/solution.h"
#include "general/random.h"
#include <vector>
#include <algorithm>
#include <numeric>

std::vector<CustomerSlot> collect_customer_slots(const Solution &sol)
{
    std::vector<CustomerSlot> slots;
    int total_slots = std::max(0, (int)(sol.truck_route.size()) - 1);
    for (const DroneCollection &drone : sol.drones)
    {
        total_slots += (int)(drone.deliver_nodes.size());
    }

    slots.reserve(total_slots);
    for (int idx = 1; idx < (int)(sol.truck_route.size()); ++idx)
    {
        slots.push_back(CustomerSlot{true, idx, -1});
    }

    for (int drone = 0; drone < (int)(sol.drones.size()); ++drone)
    {
        for (int idx = 0; idx < (int)(sol.drones[drone].deliver_nodes.size()); ++idx)
        {
            slots.push_back(CustomerSlot{false, idx, drone});
        }
    }

    return slots;
}

int read_customer_at_slot(const Solution &sol, const CustomerSlot &slot)
{
    if (slot.on_truck)
    {
        return sol.truck_route[slot.index];
    }

    return sol.drones[slot.drone].deliver_nodes[slot.index];
}

void write_customer_at_slot(Solution &sol, const CustomerSlot &slot, int customer)
{
    if (slot.on_truck)
    {
        sol.truck_route[slot.index] = customer;
        return;
    }

    sol.drones[slot.drone].deliver_nodes[slot.index] = customer;
}

std::vector<int> sample_slot_indices(int slot_count, int sample_size)
{
    std::vector<int> indices(slot_count);
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), gen);
    indices.resize(sample_size);
    return indices;
}

std::vector<int> sample_contiguous_slot_indices(int slot_count, int sample_size)
{
    if (slot_count <= 0 || sample_size <= 0 || sample_size > slot_count)
    {
        return {};
    }

    const int start_idx = rand_int(0, slot_count - sample_size);
    std::vector<int> indices(sample_size);
    std::iota(indices.begin(), indices.end(), start_idx);
    return indices;
}

std::vector<std::vector<int>> get_all_contiguous_indices(int slot_count, int sample_size) {
    if (slot_count <= 0 || sample_size <= 0 || sample_size > slot_count)
    {
        return {};
    }

    std::vector<std::vector<int>> result;

    for (int i = 0; i <= slot_count - sample_size; i++) {
        std::vector<int> new_candidates(sample_size);
        std::iota(new_candidates.begin(), new_candidates.end(), i);
        result.push_back(new_candidates);
    }

    return result;
}
