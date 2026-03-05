#include "general/sort_drone_collection.h"
#include "verification/solution.h"
#include <numeric>
#include <algorithm>
void sort_drone_collection(DroneCollection &drones)
{
    size_t n = drones.launch_indices.size();

    // Create index array
    std::vector<size_t> indices(n);
    std::iota(indices.begin(), indices.end(), 0);

    // Sort indices based on launch_indices values
    std::sort(indices.begin(), indices.end(),
              [&drones](size_t a, size_t b)
              {
                  return drones.launch_indices[a] < drones.launch_indices[b];
              });

    // Permute all three vectors using sorted indices
    std::vector<int> sorted_launch(n), sorted_deliver(n), sorted_land(n);
    for (size_t i = 0; i < n; ++i)
    {
        sorted_launch[i] = drones.launch_indices[indices[i]];
        sorted_deliver[i] = drones.deliver_nodes[indices[i]];
        sorted_land[i] = drones.land_indices[indices[i]];
    }

    drones.launch_indices = sorted_launch;
    drones.deliver_nodes = sorted_deliver;
    drones.land_indices = sorted_land;
}