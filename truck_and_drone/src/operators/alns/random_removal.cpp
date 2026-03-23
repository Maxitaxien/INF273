#include "operators/alns/random_removal.h"
#include "datahandling/instance.h"
#include "general/random.h"
#include "operators/helpers.h"
#include "verification/solution.h"
#include <vector>

std::pair<bool, std::vector<int>> random_removal(const Instance &inst, Solution &sol, int n)
{
    (void)inst;

    std::vector<int> removed;
    removed.reserve(n);

    for (int iter = 0; iter < n; ++iter)
    {
        std::vector<int> removable_segments;

        if (sol.truck_route.size() > 1)
        {
            removable_segments.push_back(0);
        }

        for (int drone = 0; drone < static_cast<int>(sol.drones.size()); ++drone)
        {
            if (!sol.drones[drone].deliver_nodes.empty())
            {
                removable_segments.push_back(drone + 1);
            }
        }

        if (removable_segments.empty())
        {
            return {false, removed};
        }

        const int chosen_segment =
            removable_segments[rand_int(0, static_cast<int>(removable_segments.size()) - 1)];

        if (chosen_segment == 0)
        {
            const int truck_idx = rand_int(1, static_cast<int>(sol.truck_route.size()) - 1);
            removed.push_back(pop_truck_delivery(sol, truck_idx));
            continue;
        }

        const int drone = chosen_segment - 1;
        const int drone_idx = rand_int(0, static_cast<int>(sol.drones[drone].deliver_nodes.size()) - 1);
        removed.push_back(sol.drones[drone].deliver_nodes[drone_idx]);
        remove_drone_flight(sol, drone, drone_idx);
    }

    return {true, removed};
}
