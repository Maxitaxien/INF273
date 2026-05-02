#include "operators/drone_rendezvous_shift.h"
#include "operators/operator.h"
#include "runners/run_algorithm.h"

#include "datahandling/datasets.h"

#include <string>
#include <vector>

namespace
{
std::vector<NamedOperator> build_ops(bool include_nn, bool include_k_medium)
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
    ops.push_back(NamedOperator{"Exchange-K Small", exchange_k_small});
    if (include_k_medium)
    {
        ops.push_back(NamedOperator{"Exchange-K Medium", exchange_k_medium});
    }
    return ops;
}

std::vector<double> build_weights(bool include_nn, bool include_k_medium)
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
    weights.push_back(0.10); // Exchange-K Small
    if (include_k_medium)
    {
        weights.push_back(0.10); // Exchange-K Medium
    }
    return weights;
}

GAMConfig build_config(bool include_nn, bool include_k_medium)
{
    GAMConfig config;
    config.phase_one_fraction = 0.5;
    config.reset_acceptance_each_phase = true;
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

    config.phase_two_operator_names = {
        "Exchange-K Small",
        "Truck replacement greedy",
        "Drone rendezvous shift best improvement",
    };
    if (include_k_medium)
    {
        config.phase_two_operator_names.insert(
            config.phase_two_operator_names.begin() + 1,
            "Exchange-K Medium");
    }

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
            "Exp 1 - SA base, no K-medium, late truck replacement",
            build_ops(true, false),
            build_weights(true, false),
            build_config(true, false),
        },
        {
            "Exp 2 - SA base, no NN, late truck replacement",
            build_ops(false, true),
            build_weights(false, true),
            build_config(false, true),
        },
        {
            "Exp 3 - SA base, no NN or K-medium, late truck replacement",
            build_ops(false, false),
            build_weights(false, false),
            build_config(false, false),
        },
    };

    run_gam_experiments(experiments, datasets, 5);
}
