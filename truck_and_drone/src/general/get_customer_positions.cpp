#include "general/get_customer_positions.h"

std::unordered_map<int, int> get_customer_positions(const Solution &solution)
{
    std::unordered_map<int, int> positions;
    for (int i = 0; i < (int)(solution.truck_route.size()); ++i)
    {
        positions[solution.truck_route[i]] = i;
    }

    return positions;
}

