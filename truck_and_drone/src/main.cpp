#include "operators/alns/alns_composite.h"
#include "operators/operator.h"
#include "runners/run_algorithm.h"
#include <vector>

int main()
{
    const std::vector<NamedOperator> ops = {
        NamedOperator{"Truck replacement random", replace_truck_delivery_random},
        NamedOperator{"Drone replacement random", replace_drone_delivery_random},
        NamedOperator{"Two-Opt Greedy", two_opt_greedy},
        NamedOperator{"Three-Opt", three_opt_random},
        NamedOperator{"NN-Reassign", nearest_neighbour_reassign_random},
        NamedOperator{"Drone planner", drone_planner_improve},
    };

    // Keep the planner in the mix as a low-frequency intensifier.
    const std::vector<double> weights = {
        0.6,
        0.4,
        0.8,
        0.5,
        1.0,
        0.08,
    };

    run_gam(ops, weights);
}
