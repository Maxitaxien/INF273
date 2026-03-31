#include "operators/operator.h"
#include <utility>
#include <vector>

Operator make_alns_operator(const ALNSOperator &op)
{
    return [op](const Instance &instance, Solution &sol) {
        Solution candidate = sol;

        if (!op.removal || !op.insertion)
        {
            return false;
        }

        const auto [removed_ok, removed_nodes] =
            op.removal(instance, candidate, op.neighbourhood_size);
        if (!removed_ok)
        {
            return false;
        }

        if (!op.insertion(instance, candidate, removed_nodes, op.regret_k))
        {
            return false;
        }

        sol = std::move(candidate);
        return true;
    };
}

NamedOperator make_named_alns_operator(const NamedALNSOperator &op)
{
    return NamedOperator{op.name, make_alns_operator(op.op)};
}

std::vector<NamedOperator> combine_alns_operator_pairs(
    const std::vector<NamedRemovalHeuristic> &removals,
    const std::vector<NamedInsertionHeuristic> &insertions,
    int neighbourhood_size,
    int regret_k)
{
    std::vector<NamedOperator> combined;
    combined.reserve(removals.size() * insertions.size());

    for (const NamedRemovalHeuristic &removal : removals)
    {
        for (const NamedInsertionHeuristic &insertion : insertions)
        {
            combined.push_back(make_named_alns_operator({
                removal.name + " + " + insertion.name,
                ALNSOperator{removal.op, insertion.op, neighbourhood_size, regret_k},
            }));
        }
    }

    return combined;
}
