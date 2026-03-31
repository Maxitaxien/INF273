#include "operators/alns/alns_composite.h"
#include "operators/operator.h"
#include "operators/drone_rendezvous_shift.h"
#include "runners/run_algorithm.h"
#include <vector>

int main()
{
    const std::vector<NamedOperator> ops = {
        NamedOperator{"NN-Reassign", nearest_neighbour_reassign_random},
        NamedOperator{"Two-Opt Greedy", two_opt_greedy},
        // NamedOperator{"Two-Opt Random", two_opt_random},
        NamedOperator{"Truck replacement greedy", replace_truck_delivery_greedy},
        //NamedOperator{"Three-Opt", three_opt_random},
        // NamedOperator{"Drone planner", drone_planner_improve},
        // NamedOperator{"Drone replacement greedy", replace_drone_delivery_greedy},
        // NamedOperator{"Drone demotion shake", drone_demotion_shake},
        NamedOperator{"Drone rendezvous shift first improvement", drone_rendezvous_shift_first_improvement}
    };

    // Keep the stronger truck-route operators active while letting the
    // paper-style random 2-opt/3-opt operators diversify the customer order.
    const std::vector<double> weights = {
        0.20, // NN-Reassign
        1.10, // Two-Opt Greedy
        // 0.30, // Two-Opt Random
        0.80, // Truck replacement greedy
        // 0.12, // Three-Opt
        // 0.15, // Drone planner
        // 0.10, // Drone replacement greedy
        // 0.20, // Drone demotion shake
        0.40, // Drone rendezvous shift
    };
    run_gam(ops, weights);
}
