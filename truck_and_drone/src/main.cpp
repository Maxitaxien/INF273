#include "operators/drone_rendezvous_shift.h"
#include "operators/operator.h"
#include "runners/run_algorithm.h"

#include "datahandling/datasets.h"

#include <string>
#include <vector>

namespace
{
std::vector<NamedOperator> build_ops(bool include_nn, Operator exact_segment_reopt_operator)
{
    std::vector<NamedOperator> ops;
    if (include_nn)
    {
        ops.push_back(NamedOperator{"NN-Reassign", nearest_neighbour_reassign_random});
    }
    ops.push_back(NamedOperator{"Two-Opt Arrival Screened", two_opt_arrival_screened});
    ops.push_back(NamedOperator{"Truck replacement greedy", replace_truck_delivery_greedy});
    ops.push_back(NamedOperator{"Targeted drone-to-truck", replace_drone_delivery_targeted});
    ops.push_back(NamedOperator{"Drone rendezvous shift best improvement", drone_rendezvous_shift_best_improvement});
    ops.push_back(NamedOperator{"Exact segment reopt", exact_segment_reopt_operator});
    return ops;
}

std::vector<double> build_weights(bool include_nn)
{
    std::vector<double> weights;
    if (include_nn)
    {
        weights.push_back(0.80); // NN-Reassign
    }
    weights.push_back(0.70); // Two-Opt Arrival Screened
    weights.push_back(0.55); // Truck replacement greedy
    weights.push_back(0.15); // Targeted drone-to-truck
    weights.push_back(0.08); // Drone rendezvous shift best improvement
    weights.push_back(0.40); // Exact segment reopt
    return weights;
}

GAMConfig build_config(bool include_nn)
{
    GAMConfig config;
    config.phase_one_fraction = 1;
    config.acceptance_mode = GAMAcceptanceMode::SimulatedAnnealing;
    config.escape_mode = GAMEscapeMode::ExchangeKLarge;

    // B = Exp 4 is the chosen base. Phase 1 keeps the Apr16-style SA line,
    // while phase 2 is simplified because K-medium was mostly inactive and
    // Truck replacement greedy remains the strongest structural improver.
    if (include_nn)
    {
        config.phase_one_operator_names.push_back("NN-Reassign");
    }
    config.phase_one_operator_names.push_back("Two-Opt Arrival Screened");
    config.phase_one_operator_names.push_back("Truck replacement greedy");
    config.phase_one_operator_names.push_back("Targeted drone-to-truck");
    config.phase_one_operator_names.push_back("Drone rendezvous shift best improvement");
    config.phase_one_operator_names.push_back("Exact segment reopt");

    return config;
}
}

int main()
{
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

    const std::vector<GAMExperiment> experiments = {
        {
            "Exp 1 - NN + exact segment reopt random small",
            build_ops(true, exact_segment_reopt_random_small),
            build_weights(true),
            build_config(true),
        },
        {
            "Exp 2 - No NN + exact segment reopt random small",
            build_ops(false, exact_segment_reopt_random_small),
            build_weights(false),
            build_config(false),
        },
        {
            "Exp 3 - NN + exact segment reopt random medium",
            build_ops(true, exact_segment_reopt_random_medium),
            build_weights(true),
            build_config(true),
        },
        {
            "Exp 4 - No NN + exact segment reopt random medium",
            build_ops(false, exact_segment_reopt_random_medium),
            build_weights(false),
            build_config(false),
        },
        {
            "Exp 5 - NN + exact segment reopt random large",
            build_ops(true, exact_segment_reopt_random_large),
            build_weights(true),
            build_config(true),
        },
        {
            "Exp 6 - No NN + exact segment reopt random large",
            build_ops(false, exact_segment_reopt_random_large),
            build_weights(false),
            build_config(false),
        },
    };

    run_gam_experiments(experiments, datasets, 1);
}
