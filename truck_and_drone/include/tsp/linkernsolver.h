#pragma once

#include <vector>

struct ConcordeLinkernHandle;

class LinkernSolver
{
public:
    explicit LinkernSolver(const std::vector<std::vector<long long>> &dist, int seed = 1);
    ~LinkernSolver();

    LinkernSolver(const LinkernSolver &) = delete;
    LinkernSolver &operator=(const LinkernSolver &) = delete;

    double improve(std::vector<int> &tour,
                   int stallcount = 10,
                   int repeatcount = 20,
                   int kicktype = 0,
                   double time_bound = -1.0);

private:
    int n_ = 0;
    ConcordeLinkernHandle *handle_ = nullptr;
};