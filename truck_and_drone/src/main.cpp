#include "operators/alns/alns_composite.h"
#include "operators/operator.h"
#include "runners/run_algorithm.h"
#include <vector>

int main()
{
    const std::vector<NamedOperator> ops = {
        NamedOperator{"NN-Reassign", nearest_neighbour_reassign_random},
        NamedOperator{"Two-Opt Greedy", two_opt_greedy},
        NamedOperator{"Truck replacement greedy", replace_truck_delivery_greedy},
        NamedOperator{"Drone replacement greedy", replace_drone_delivery_greedy},
        NamedOperator{"Drone demotion shake", drone_demotion_shake},
        NamedOperator{"Three-Opt", three_opt_random},
        NamedOperator{"Drone planner", drone_planner_improve},
    };

    // Keep the default GAM mix close to the stronger LS/SA backbone.
    // Use 3-opt and the planner only as low-frequency intensifiers.
    const std::vector<double> weights = {
        0.55, // NN-Reassign
        1.20, // Two-Opt Greedy
        1.00, // Truck replacement greedy
        0.85, // Drone replacement greedy
        0.55, // Drone demotion shake
        0.45, // Three-Opt
        0.25, // Drone planner
    };
    run_gam(ops, weights);
}
