#pragma once

#include <vector>

struct ConcordeLinkernHandle;

struct LinkernTour
{
    std::vector<int> tour;
    double value = 0.0;
};

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

    LinkernTour solve(int stallcount = 10,
                      int repeatcount = 20,
                      int kicktype = 0,
                      double time_bound = -1.0);

    LinkernTour solve_route(const std::vector<int> &route_nodes,
                            int start_node = -1,
                            int stallcount = 10,
                            int repeatcount = 20,
                            int kicktype = 0,
                            double time_bound = -1.0) const;

    double tour_length(const std::vector<int> &tour) const;

private:
    void ensure_handle();

    std::vector<int> rotate_cycle_to_start(const std::vector<int> &tour,
                                           int start_node) const;

    int n_ = 0;
    int seed_ = 1;
    std::vector<std::vector<long long>> dist_;
    std::vector<int> flat_dist_;
    ConcordeLinkernHandle *handle_ = nullptr;
};
