#include "operators/alns/alns_composite.h"
#include "operators/operator.h"
#include "runners/run_algorithm.h"
#include <vector>

int main()
{
    const std::vector<NamedOperator> ops = {
        NamedOperator{"Truck replacement random", replace_truck_delivery_random},
        NamedOperator{"Drone replacement random", replace_drone_delivery_random},
        NamedOperator{"Two-Opt", two_opt_random},
        NamedOperator{"Three-Opt", three_opt_random},
        NamedOperator{"NN-Reassign", nearest_neighbour_reassign_random},
        NamedOperator{"Drone planner", drone_planner_improve},
    };

    // Keep the planner in the mix as a low-frequency intensifier.
    const std::vector<double> weights = {
        1.0,
        1.0,
        0.5,
        0.25,
        0.05,
    };

    run_gam(ops, weights);
}
