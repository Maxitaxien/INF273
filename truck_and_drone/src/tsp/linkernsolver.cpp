#include "tsp/linkernsolver.h"
#include "tsp/concorde_linkern_c_api.h"

#include <algorithm>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

LinkernSolver::LinkernSolver(const std::vector<std::vector<long long>> &dist, int seed)
    : seed_(seed),
      dist_(dist)
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

    flat_dist_.resize((size_t)n_ * (size_t)n_);
    for (int i = 0; i < n_; ++i)
    {
        for (int j = 0; j < n_; ++j)
        {
            const long long value = dist[i][j];
            if (value < 0 || value > std::numeric_limits<int>::max())
            {
                throw std::invalid_argument("LinkernSolver: distances must fit in a non-negative int");
            }

            flat_dist_[(size_t)i * n_ + j] = static_cast<int>(value);
        }
    }
}

LinkernSolver::~LinkernSolver()
{
    concorde_linkern_destroy(handle_);
    handle_ = nullptr;
}

void LinkernSolver::ensure_handle()
{
    if (handle_)
    {
        return;
    }

    handle_ = concorde_linkern_create(n_, flat_dist_.data(), seed_);
    if (!handle_)
    {
        throw std::runtime_error("LinkernSolver: failed to initialize Concorde handle");
    }
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

    if (n_ < 10 && repeatcount > 0)
    {
        repeatcount = 0;
    }

    ensure_handle();

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

LinkernTour LinkernSolver::solve(int stallcount,
                                 int repeatcount,
                                 int kicktype,
                                 double time_bound)
{
    std::vector<int> tour(n_);
    std::iota(tour.begin(), tour.end(), 0);

    const double value = improve(tour, stallcount, repeatcount, kicktype, time_bound);
    return LinkernTour{std::move(tour), value};
}

LinkernTour LinkernSolver::solve_route(const std::vector<int> &route_nodes,
                                       int start_node,
                                       int stallcount,
                                       int repeatcount,
                                       int kicktype,
                                       double time_bound) const
{
    if (route_nodes.empty())
    {
        throw std::invalid_argument("LinkernSolver::solve_route: empty route");
    }

    std::vector<bool> seen((size_t)n_, false);
    for (const int node : route_nodes)
    {
        if (node < 0 || node >= n_)
        {
            throw std::invalid_argument("LinkernSolver::solve_route: node out of range");
        }
        if (seen[(size_t)node])
        {
            throw std::invalid_argument("LinkernSolver::solve_route: duplicate nodes are not allowed");
        }
        seen[(size_t)node] = true;
    }

    std::vector<int> mapped_tour = route_nodes;
    if ((int)route_nodes.size() >= 3)
    {
        const int route_size = static_cast<int>(route_nodes.size());
        std::vector<std::vector<long long>> submatrix(
            (size_t)route_size,
            std::vector<long long>((size_t)route_size, 0));

        for (int i = 0; i < route_size; ++i)
        {
            for (int j = 0; j < route_size; ++j)
            {
                submatrix[(size_t)i][(size_t)j] = dist_[(size_t)route_nodes[(size_t)i]][(size_t)route_nodes[(size_t)j]];
            }
        }

        LinkernSolver subsolver(submatrix, seed_);
        std::vector<int> local_tour(route_size);
        std::iota(local_tour.begin(), local_tour.end(), 0);
        subsolver.improve(local_tour, stallcount, repeatcount, kicktype, time_bound);

        for (int i = 0; i < route_size; ++i)
        {
            mapped_tour[(size_t)i] = route_nodes[(size_t)local_tour[(size_t)i]];
        }
    }

    if (start_node >= 0)
    {
        const auto open_path_length = [this](const std::vector<int> &path) {
            double total = 0.0;
            for (size_t i = 1; i < path.size(); ++i)
            {
                total += static_cast<double>(dist_[(size_t)path[i - 1]][(size_t)path[i]]);
            }
            return total;
        };

        const auto positional_mismatch = [&route_nodes](const std::vector<int> &path) {
            int mismatch = 0;
            for (size_t i = 0; i < path.size() && i < route_nodes.size(); ++i)
            {
                if (path[i] != route_nodes[i])
                {
                    ++mismatch;
                }
            }
            return mismatch;
        };

        const std::vector<int> forward = rotate_cycle_to_start(mapped_tour, start_node);

        std::vector<int> reversed_cycle = mapped_tour;
        std::reverse(reversed_cycle.begin(), reversed_cycle.end());
        const std::vector<int> reversed = rotate_cycle_to_start(reversed_cycle, start_node);

        const double forward_length = open_path_length(forward);
        const double reversed_length = open_path_length(reversed);

        if (reversed_length < forward_length)
        {
            mapped_tour = reversed;
        }
        else if (forward_length < reversed_length)
        {
            mapped_tour = forward;
        }
        else
        {
            mapped_tour =
                positional_mismatch(reversed) < positional_mismatch(forward) ? reversed : forward;
        }
    }

    return LinkernTour{mapped_tour, tour_length(mapped_tour)};
}

double LinkernSolver::tour_length(const std::vector<int> &tour) const
{
    if (tour.empty())
    {
        return 0.0;
    }

    double total = 0.0;
    for (size_t i = 1; i < tour.size(); ++i)
    {
        const int from = tour[i - 1];
        const int to = tour[i];
        if (from < 0 || from >= n_ || to < 0 || to >= n_)
        {
            throw std::invalid_argument("LinkernSolver::tour_length: node out of range");
        }
        total += static_cast<double>(dist_[(size_t)from][(size_t)to]);
    }

    const int last = tour.back();
    const int first = tour.front();
    if (last < 0 || last >= n_ || first < 0 || first >= n_)
    {
        throw std::invalid_argument("LinkernSolver::tour_length: node out of range");
    }

    total += static_cast<double>(dist_[(size_t)last][(size_t)first]);
    return total;
}

std::vector<int> LinkernSolver::rotate_cycle_to_start(const std::vector<int> &tour,
                                                      int start_node) const
{
    const auto start_it = std::find(tour.begin(), tour.end(), start_node);
    if (start_it == tour.end())
    {
        throw std::invalid_argument("LinkernSolver::rotate_cycle_to_start: start node not in cycle");
    }

    std::vector<int> rotated;
    rotated.reserve(tour.size());
    rotated.insert(rotated.end(), start_it, tour.end());
    rotated.insert(rotated.end(), tour.begin(), start_it);
    return rotated;
}
