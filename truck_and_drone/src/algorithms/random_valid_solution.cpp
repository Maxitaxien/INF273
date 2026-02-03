#include "algorithms/random_valid_solution.h"
#include <algorithm>

const int SEED = 42;
const int DRONES = 2;

template <typename T>
std::vector<T> random_sample(const std::vector<T>& vec, size_t k) {
    if (k > vec.size()) throw std::runtime_error("Sample size larger than vector size");

    std::vector<T> copy = vec; 
    std::random_device rd;
    std::mt19937 gen(SEED);

    std::shuffle(copy.begin(), copy.end(), gen);
    return std::vector<T>(copy.begin(), copy.begin() + k); 
}


template <typename T>
std::vector<std::vector<T>> random_split(const std::vector<T>& vec, size_t k) {
    if (k > vec.size())
        throw std::runtime_error("Sample size larger than vector size");

    std::vector<T> copy = vec;
    std::random_device rd;
    std::mt19937 gen(SEED);

    std::shuffle(copy.begin(), copy.end(), gen);

    std::vector<std::vector<T>> result;
    result.emplace_back(copy.begin(), copy.begin() + k);
    result.emplace_back(copy.begin() + k, copy.end());

    return result;
}

// Note: Make sure not to assign drone launch -> land sequences in such a way that it is impossible to assign all drones.
// To fix this, we could start from the back - assign the landing spaces first as a random selection.
// Then, assign the start spaces.
// This will always work even in the "worst" case: Let's say we assign all values at the start of the tour to land.
// Then, simply assign 0 to launch, 1 to launch on next, ... -> n to launch and land on n + 1
Solution random_valid_solution(int n) {
    Solution random_solution;

    // ----- Truck route generation -----
    std::vector<bool> available(n + 1, true);
    std::mt19937 gen(SEED);
    std::uniform_int_distribution<> dist((int) ((n / 3) + 1), n);

    // Create list of available nodes
    std::vector<int> nodes(n);
    std::iota(std::begin(nodes), std::end(nodes), 1);
    
    random_solution.truck_route.push_back(0); // start from depot in all cases
    available[0] = false;

    std::vector<int> add_to_truck = random_sample(nodes, dist(gen));
    
    for (int val : add_to_truck) {
        random_solution.truck_route.push_back(val);
        available[val] = false;
    }

    // ----- Drone generation -----
    // Drone coverings: Get remaining nodes, shuffle and give first k to first drone, rest to second drone
    std::vector<int> remaining;
    for (int val: nodes) {
        if (available[val]) {
            remaining.push_back(val);
        }
    }

    std::vector<std::vector<int>> assigned;
    assigned = random_split(remaining, dist(gen));

    // Generate random permutation of landing spots for each drone customer serving
    // Then, generate launching spots conditionally on this
    std::vector<bool> is_available_for_drone[DRONES];
    std::mt19937 gen(SEED);
    std::uniform_int_distribution<> dist((int) ((n / 3) + 1), n);

    for (std::vector<int> drone_cover : assigned) {
        for (int i : drone_cover) {

        }
    }









    random_solution.truck_route.push_back(0); // end at depot in all cases
}