#include "datahandling/reader.h"
#include <fstream>
#include <iostream>
#include <sstream>

Instance read_instance(const std::string& filename) {
    std::ifstream f(filename);

    if (!f) {
        std::cerr << "Error: Unable to open file: " << filename << std::endl;
    }

    std::string line;
    Instance problem_instance;

    // Number of drones
    std::getline(f, line); // skip header
    std::getline(f, line);
    problem_instance.n = std::stoi(line);

    // Drone flight limit
    std::getline(f, line); // skip header
    std::getline(f, line);
    problem_instance.lim = std::stoi(line);

    // Truck travel matrix
    std::getline(f, line);
    std::vector<std::vector<long long>> truck_matrix;
    for (int i = 0; i < problem_instance.n + 1; i++) {
        std::getline(f, line);
        std::istringstream ss(line);
        std::vector<long long> row;
        double x;
        while (ss >> x) {
            row.push_back(static_cast<long long>(x));
        }
        truck_matrix.push_back(row);
    }
    problem_instance.truck_matrix = truck_matrix;

    // Drone travel matrix
    std::getline(f, line);
    std::vector<std::vector<long long>> drone_matrix;
    double y;
    for (int i = 0; i < problem_instance.n + 1; ++i) {
        std::getline(f, line);
        std::istringstream ss(line);
        std::vector<long long> row;
        while (ss >> y) {
            row.push_back(static_cast<long long>(y));
        }
        if ((int)(row.size()) != problem_instance.n + 1) {
            std::cerr << "Error: row " << i << " has " << row.size()
                      << " columns, expected " << problem_instance.n + 1 << "\n";
        }
        drone_matrix.push_back(row);
    }

    problem_instance.drone_matrix = drone_matrix;

    return problem_instance;
}

