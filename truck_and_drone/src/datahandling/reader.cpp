#include "datahandling/reader.h"
#include <fstream>
#include <iostream>

Instance read_instance(const std::string& filename) {
    Instance i;
    std::ifstream f(filename);

    if (!f) {
        std::cerr << "Error: Unable to open file: " << filename << std::endl;
    }

    std::string txt;
    getline(f, txt); // skip header

    // Number of drones
    int n;
    f >> n;

    // 



    return Instance();
}