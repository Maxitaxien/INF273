#include "operators/alns/alns_composite.h"
#include "operators/alns/signatures.h"
#include "general/roulette_wheel_selection.h"

#include <vector>

struct AdaptiveALNSCompositeOperator
{
    std::vector<RemovalHeuristic> removal_ops;
    std::vector<InsertionHeuristic> insertion_ops;

    std::vector<double> removal_weights;
    std::vector<double> insertion_weights;

    int iteration = 0;
    int segment_length = 100;

    int current_n;

    void normalize_weights(std::vector<double> weights) {

    }

    bool operator()(const Instance &inst, Solution &sol)
    {
        iteration++;

        int r = roulette_wheel_selection(removal_weights);
        int i = roulette_wheel_selection(insertion_weights);

        std::pair<bool, std::vector<int>> removed = removal_ops[r](inst, sol, current_n);
        if (!removed.first)
            return false;

        bool inserted = insertion_ops[i](inst, sol, removed.second, 0); // call with k = 0 always for now
        if (!inserted)
            return false;

        if (iteration % segment_length == 0)
        {
            // TODO: update weights

        }

        return true;
    }
};