#include "operators/alns/alns_composite.h"
#include "operators/operator.h"
#include "runners/run_algorithm.h"

int main()
{
    run_gam(
        {
            NamedOperator{
                "ALNS Composite Random Removal + Greedy Insert",
                AdaptiveCompositeOperator(
                    alns_heuristic::removal,
                    alns_heuristic::insertion)},
            NamedOperator{"Replace Truck Delivery", replace_truck_delivery_greedy},
            NamedOperator{"Two-Opt", two_opt_random},
            NamedOperator{"NN-Reassign", nearest_neighbour_reassign_random},
        }
    );
}
