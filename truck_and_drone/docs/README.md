# README

This directory contains some documentation for the current and best-performing mix of operators, as well as documentation for the in-use solution fixers and final algorithm version (`algorithms/gam.cpp`)

Also in this directory

## Building/running the project

This project includes all dependencies natively.

Run the following commands inside build-native for a fast optimized build version:

On creating the project:
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j${nproc}
./truck_and_drone
