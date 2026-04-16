#include "tsp/linkernsolver.h"
#include "tsp/concorde_linkern_c_api.h"

#include <stdexcept>
#include <vector>

LinkernSolver::LinkernSolver(const std::vector<std::vector<long long>> &dist, int seed)
{
    n_ = static_cast<int>(dist.size());
    if (n_ == 0)
    {
        throw std::invalid_argument("LinkernSolver: empty distance matrix");
    }

    for (const auto &row : dist)
    {
        if (static_cast<int>(row.size()) != n_)
        {
            throw std::invalid_argument("LinkernSolver: distance matrix must be square");
        }
    }

    std::vector<int> flat((size_t)n_ * (size_t)n_);
    for (int i = 0; i < n_; ++i)
    {
        for (int j = 0; j < n_; ++j)
        {
            flat[(size_t)i * n_ + j] = static_cast<int>(dist[i][j]);
        }
    }

    handle_ = concorde_linkern_create(n_, flat.data(), seed);
    if (!handle_)
    {
        throw std::runtime_error("LinkernSolver: failed to initialize Concorde handle");
    }
}

LinkernSolver::~LinkernSolver()
{
    concorde_linkern_destroy(handle_);
    handle_ = nullptr;
}

double LinkernSolver::improve(std::vector<int> &tour,
                              int stallcount,
                              int repeatcount,
                              int kicktype,
                              double time_bound)
{
    if (static_cast<int>(tour.size()) != n_)
    {
        throw std::invalid_argument("LinkernSolver::improve: wrong tour size");
    }

    double val = 0.0;
    int rval = concorde_linkern_improve(
        handle_,
        tour.data(),
        stallcount,
        repeatcount,
        kicktype,
        time_bound,
        &val);

    if (rval != 0)
    {
        throw std::runtime_error("LinkernSolver::improve: concorde_linkern_improve failed");
    }

    return val;
}