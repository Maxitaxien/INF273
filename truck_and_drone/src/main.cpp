#include "operators/drone_rendezvous_shift.h"
#include "operators/operator.h"
#include "runners/run_algorithm.h"

#include "datahandling/datasets.h"

#include <vector>

int main()
{
    const std::vector<NamedOperator> ops = {
        NamedOperator{"NN-Reassign", nearest_neighbour_reassign_random},
        NamedOperator{"Two-Opt Arrival Screened", two_opt_arrival_screened},
        NamedOperator{"Truck replacement greedy", replace_truck_delivery_greedy},
        NamedOperator{"Targeted drone-to-truck", replace_drone_delivery_targeted},
        NamedOperator{"Drone rendezvous shift best improvement", drone_rendezvous_shift_best_improvement},
        NamedOperator{"Exchange-K Small", exchange_k_small},
        NamedOperator{"Exchange-K Medium", exchange_k_medium},
    };

    const std::vector<double> weights = {
        0.80, // NN-Reassign
        0.70, // Two-Opt Arrival Screened
        0.55, // Truck replacement greedy
        0.15, // Targeted drone-to-truck
        0.08, // Drone rendezvous shift best improvement
        0.10, // Exchange-K Small
        0.10, // Exchange-K Medium
    };

    const std::vector<std::string> datasets = {
        datasets::f10,
        datasets::f20,
        datasets::f50,
        datasets::f100,
        datasets::r10,
        datasets::r20,
        datasets::r50,
        datasets::r100,
    };

    GAMConfig base_config;
    base_config.phase_one_fraction = 0.5;
    base_config.reset_acceptance_each_phase = true;
    // Phase 1 keeps the restored Apr16 mix intact.
    base_config.phase_one_operator_names = {
        "NN-Reassign",
        "Two-Opt Arrival Screened",
        "Truck replacement greedy",
        "Targeted drone-to-truck",
        "Drone rendezvous shift best improvement",
    };
    // Phase 2 narrows to smaller disruptive exchange-k neighborhoods plus
    // local drone launch/land reassignment.
    base_config.phase_two_operator_names = {
        "Exchange-K Small",
        "Exchange-K Medium",
        "Drone rendezvous shift best improvement",
    };

    GAMConfig rrt_config = base_config;
    rrt_config.acceptance_mode = GAMAcceptanceMode::BestRelativeRRT;

    GAMConfig rrt_escape_config = rrt_config;
    // Exchange-K Large is reserved for escape only in this experiment family.
    rrt_escape_config.escape_mode = GAMEscapeMode::ExchangeKLarge;

    GAMConfig sa_config = base_config;
    sa_config.acceptance_mode = GAMAcceptanceMode::SimulatedAnnealing;

    GAMConfig sa_escape_config = sa_config;
    sa_escape_config.escape_mode = GAMEscapeMode::ExchangeKLarge;

    const std::vector<GAMExperiment> experiments = {
        {
            "Exp 1 - RRT, late K-small + K-medium + drone shift",
            ops,
            weights,
            rrt_config,
        },
        {
            "Exp 2 - RRT, late K-small + K-medium + drone shift, K-large escape",
            ops,
            weights,
            rrt_escape_config,
        },
        {
            "Exp 3 - SA, late K-small + K-medium + drone shift",
            ops,
            weights,
            sa_config,
        },
        {
            "Exp 4 - SA, late K-small + K-medium + drone shift, K-large escape",
            ops,
            weights,
            sa_escape_config,
        },
    };

    run_gam_experiments(experiments, datasets, 5);
}
