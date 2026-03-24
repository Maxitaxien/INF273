#include "operators/alns/alns_composite.h"
#include "operators/operator.h"
#include "runners/run_algorithm.h"
#include <vector>

int main()
{
    /**
    * NamedOperator{
    * "ALNS Composite Random Removal + Cheapest Sequential Insert",
    * AdaptiveCompositeOperator(
    *    alns_heuristic::removal,
    *    alns_heuristic::insertion_sequential)},
    */
    run_gam(
        {
            NamedOperator{"1-reinsert", one_reinsert_random},
            NamedOperator{"Replace Truck Delivery", replace_truck_delivery_greedy},
            NamedOperator{"Two-Opt", two_opt_random},
            NamedOperator{"NN-Reassign", nearest_neighbour_reassign_random},
        }
    );
}
