#include "operators/drone_rendezvous_shift.h"
#include "operators/operator.h"
#include "runners/run_algorithm.h"
#include <vector>

#include "datahandling/datasets.h"

int main()
{
    const std::vector<NamedOperator> ops = {
        NamedOperator{"NN-Reassign", nearest_neighbour_reassign_random},
        NamedOperator{"Two-Opt Arrival Screened", two_opt_arrival_screened},
        NamedOperator{"Truck replacement greedy", replace_truck_delivery_greedy},
        NamedOperator{"Targeted drone-to-truck", replace_drone_delivery_targeted},
        NamedOperator{"Drone rendezvous shift best improvement", drone_rendezvous_shift_best_improvement},
        NamedOperator{
            "Concorde K-Swap Rebuild",
            static_cast<bool (*)(const Instance &, Solution &)>(concorde_k_swap_rebuild)},
    };

    const std::vector<double> weights = {
        0.80, // NN-Reassign
        0.70, // Two-Opt Arrival Screened
        0.55, // Truck replacement greedy
        0.15, // Targeted drone-to-truck
        0.08, // Drone rendezvous shift best improvement
        0.10, // Concorde K-Swap Rebuild
    };

    GAMConfig config;
    config.phase_one_fraction = 0.5;
    config.phase_one_operator_names = {
        "NN-Reassign",
        "Two-Opt Arrival Screened",
        "Truck replacement greedy",
        "Targeted drone-to-truck",
        "Drone rendezvous shift best improvement",
    };
    config.phase_two_operator_names = {
        "Concorde K-Swap Rebuild",
        "Drone rendezvous shift best improvement",
    };

    const std::vector<std::string> datasets = {
        datasets::f50,
        datasets::r50,
        datasets::f100,
        datasets::r100,
    };

    // Default benchmark: restored Apr16-style GAM for the first half,
    // then late intensification with Concorde rebuild + bounded rendezvous shifts.
    run_gam(ops, weights, config, datasets, 3);
}
