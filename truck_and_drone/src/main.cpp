#include "operators/drone_rendezvous_shift.h"
#include "operators/operator.h"
#include "runners/run_algorithm.h"

#include "datahandling/datasets.h"

#include <vector>

namespace
{
bool shaw_removal_greedy_repair_size_adaptive(
    const Instance &inst,
    Solution &sol)
{
    if (inst.n >= 50)
    {
        return shaw_removal_greedy_repair_random_large(inst, sol);
    }

    return shaw_removal_greedy_repair_random_medium(inst, sol);
}

std::vector<NamedOperator> build_ops(Operator shaw_removal_op)
{
    std::vector<NamedOperator> ops;
    ops.push_back(NamedOperator{"NN-Reassign", nearest_neighbour_reassign_random});
    ops.push_back(NamedOperator{"Two-Opt Arrival Screened", two_opt_arrival_screened});
    ops.push_back(NamedOperator{"Truck replacement greedy", replace_truck_delivery_greedy});
    ops.push_back(NamedOperator{"Targeted drone-to-truck", replace_drone_delivery_targeted});
    ops.push_back(NamedOperator{"Drone rendezvous shift best improvement", drone_rendezvous_shift_best_improvement});
    ops.push_back(NamedOperator{"Shaw removal greedy repair", shaw_removal_op});
    return ops;
}

std::vector<double> build_weights()
{
    std::vector<double> weights;
    weights.push_back(0.60); // NN-Reassign
    weights.push_back(0.50); // Two-Opt Arrival Screened
    weights.push_back(0.55); // Truck replacement greedy
    weights.push_back(0.15); // Targeted drone-to-truck
    weights.push_back(0.60); // Drone rendezvous shift best improvement
    weights.push_back(0.70); // shaw removal greedy repair, different sizes
    return weights;
}

}

int main()
{
    const std::vector<std::string> datasets = {
        datasets::f10,
        datasets::r10,
        datasets::f20,
        datasets::r20,
        datasets::f50,
        datasets::r50,
        datasets::f100,
        datasets::r100,
    };

    GAMConfig config{};
    config.enable_drastic_random_restart = true;

    
    run_gam_parallel_batch(
        build_ops(shaw_removal_greedy_repair_size_adaptive),
        build_weights(),
        config,
        datasets);

    // Full sequential benchmark example:
    // const std::vector<std::string> benchmark_datasets = {
    //     datasets::f10,
    //     datasets::f20,
    //     datasets::f50,
    //     datasets::f100,
    //     datasets::r10,
    //     datasets::r20,
    //     datasets::r50,
    //     datasets::r100,
    // };
    // const std::vector<GAMExperiment> experiments = {
    //     GAMExperiment{
    //         "Final Benchmark",
    //         build_ops(shaw_removal_greedy_repair_size_adaptive),
    //         build_weights(),
    //         config,
    //     },
    // };
    // run_gam_experiments(experiments, benchmark_datasets, 10);
}
