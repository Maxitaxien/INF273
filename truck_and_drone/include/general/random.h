#pragma once
#include <vector>
#include <random>
#include <algorithm>
#include <cstddef>


extern std::mt19937 gen;

int rand_int(int a, int b);

double rand_double(double a, double b);

template <typename T>
std::vector<T> random_sample(const std::vector<T>& vec, size_t k) {
    std::vector<T> copy = vec;
    std::shuffle(copy.begin(), copy.end(), gen);
    copy.resize(k);
    return copy;
}

template <typename T>
std::pair<std::vector<T>, std::vector<T>> random_partition(const std::vector<T>& vec, size_t k) {
    std::vector<T> copy = vec;
    std::shuffle(copy.begin(), copy.end(), gen);

    std::vector<T> first(copy.begin(), copy.begin() + k);
    std::vector<T> second(copy.begin() + k, copy.end());
    return {first, second};
}
template <typename T>
void random_shuffle(std::vector<T> &vec) {
    std::shuffle(vec.begin(), vec.end(), gen);
}

template <typename T>
T get_random(const std::vector<T> &vec) {
    int idx = rand_int(0, vec.size() - 1);
    return vec[idx];
}
