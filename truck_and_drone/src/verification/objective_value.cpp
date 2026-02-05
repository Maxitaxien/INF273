#include "verification/objective_value.h"

long long calculateTotalWaitingTime(const Instance& problem_instance, const Solution& solution) {
    long long totalWaitTime = 0;

    // Track drone arrival times at a given node - compare with truck arrival times later
    std::vector<long long> droneArrivals(problem_instance.n + 1);
    std::vector<long long> truckArrivals(problem_instance.n + 1);
    std::vector<long long> combinedArrivals(problem_instance.n + 1);
 
    // ----- Start from the truck route -----
    int prev = 0;
    int curr;
    for (int i = 1; i < solution.truck_route.size(); i++) {
        prev = solution.truck_route[i - 1];
        curr = solution.truck_route[i]; 
        // Check both drones from this launch, potentially update arrival times at future node
        for (auto [deliver, land] : solution.drone_map[curr]) {
            droneArrivals[deliver] = std::max(droneArrivals[deliver], problem_instance.drone_matrix[curr][deliver]);
            droneArrivals[land] = std::max(droneArrivals[land], 
                problem_instance.drone_matrix[curr][deliver] + problem_instance.drone_matrix[deliver][land]);    
        }
    }
}

long long getFinalArrivalTime(const Instance& problem_instance, const Solution& solution) {
    return 0L;
}