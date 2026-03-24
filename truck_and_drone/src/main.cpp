#include "operators/alns/alns_composite.h"
#include "operators/operator.h"
#include "runners/run_algorithm.h"
#include <vector>

int main()
{
    // TODO: Stop drone assignments from getting too static.
    // Problem as of now is that we assign many to drone in the start.
    // This is given a high weight as it is successful.
    // Then, when we try to remove from the drone assignments later, we cannot get out of the local opt
    // Instead, we should use a greedy assignment operator in general (similar to 1 insert)
    const std::vector<NamedOperator> ops = {
        NamedOperator{"NN-Reassign", nearest_neighbour_reassign_random},
        NamedOperator{"Two-Opt Greedy", two_opt_greedy},
        NamedOperator{"Truck replacement greedy", replace_truck_delivery_greedy},
        NamedOperator{"Drone replacement greedy", replace_drone_delivery_greedy},
        NamedOperator{"Three-Opt", three_opt_random},
        NamedOperator{"Drone planner", drone_planner_improve},
    };

    // Keep the default GAM mix close to the stronger LS/SA backbone.
    // Use 3-opt and the planner only as low-frequency intensifiers.
    const std::vector<double> weights = {
        1.0,
        1.2,
        0.7,
        0.8,
        0.2,
        0.05,
    };

    run_gam(ops, weights);
}
