#include<iostream>
#include "algorithms/nearest_neighbour.h"

const long long INF = 4e18;

Solution nearest_neighbour(const Instance& problem_instance) {
    Solution s;
    std::vector<bool> visited(problem_instance.n + 1, false);

    std::vector<int> tour(problem_instance.n + 2);
    tour[0] = 0; // always begin at depot
    visited[0] = true;

    int i = 0;


    // Build truck tour using nearest neighbour
    while (i < problem_instance.n) {
        int prev = tour[i];
        long long best = INF;
        int next_node = -1;

        for (int neighbour = 0; neighbour < problem_instance.n + 1; neighbour++) {
            long long cost = problem_instance.truck_matrix[prev][neighbour];
            if (!visited[neighbour] && cost < best) {
                best = cost;
                next_node = neighbour;
            }
        }
        
        i++;
        tour[i] = next_node;
        visited[next_node] = true;
    }

    s.truck_route = tour;

    // Build empty drone map
    s.drone_map = std::unordered_map<int, std::vector<std::pair<int,int>>>();
    
    for (int j = 0; j <= problem_instance.n; ++j) {
        s.drone_map[j] = std::vector<std::pair<int, int>>(problem_instance.m, {-1, -1});
    }

    return s;
}