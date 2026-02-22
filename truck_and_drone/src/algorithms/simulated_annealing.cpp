#include "algorithms/simulated_annealing.h"
#include "verification/solution.h"
#include "verification/feasibility_check.h"
#include "operators/operator.h"
#include "datahandling/instance.h"
#include <numeric>
#include <cmath>
#include <random>

const int SEED = 42;
std::mt19937 gen(SEED);

int randInt(int a, int b) {
    std::uniform_int_distribution<> dist(a, b);
    return dist(gen);
}

double warmup_phase(
    const Instance& instance,
    const Solution& solution,
    long long best_cost,
    Operator op,
    int warmup_amount,
    std::function<long long(const Instance&, const Solution&)> objective
) {
    Solution best = solution;
    Solution incumbent = solution;
    long long incumbent_cost = objective(instance, incumbent);

    long long delta_e;
    std::vector<long long> delta_ws; 
    delta_ws.reserve(warmup_amount);
    
    for (int w = 0; w < warmup_amount; w++) {
        Solution s = op(instance, incumbent); 

        long long cost = objective(instance, s);
        long long delta_e = cost - incumbent_cost;

        bool feasible = master_check(instance, s, false);
        if (!feasible) continue;

        if (delta_e < 0) {
            incumbent = s;
            incumbent_cost = cost;
            if (cost < best_cost) {
                best = s;
                best_cost = cost;
            }
        } else {
            std::uniform_real_distribution<double> dist(0.0, 1.0);
            if (dist(gen) < 0.8) {
                incumbent = s;
                incumbent_cost = cost; // also update cost here
            }
            delta_ws.push_back(delta_e);
        }
    }
    return std::accumulate(delta_ws.begin(), delta_ws.end(), 0.0) / delta_ws.size();
}

Solution simulated_annealing(
    const Instance& instance,
    const Solution& initial,
    Operator op,
    double TF,
    std::function<long long(const Instance&, const Solution&)> objective
) 
{
    long long best_cost = objective(instance, initial);
    int warmup_amount = 100;

    double delta_avg = warmup_phase(instance, initial, best_cost, op, warmup_amount, objective);

    Solution incumbent = initial;
    Solution best = initial;

    int amnt_iterations = 10000 - warmup_amount;

    double T0 = -delta_avg / std::log(0.8);
    double alpha = pow(TF / T0, 1.0 / amnt_iterations);

    double T = T0;

    long long delta_e;
    long long incumbent_cost = objective(instance, incumbent);
    

    for (int i = 0; i < amnt_iterations; i++) {
        Solution s = op(instance, incumbent); 

        long long cost = objective(instance, s);
        long long delta_e = cost - incumbent_cost;

        bool feasible = master_check(instance, s, false);
        if (!feasible) continue;

        if (delta_e < 0) {
            incumbent = s;
            incumbent_cost = cost;
            if (cost < best_cost) {
                best = s;
                best_cost = cost;
            }
        } else {
            double p = exp(-delta_e / T);
            std::uniform_real_distribution<double> dist(0.0, 1.0);
            if (dist(gen) < p) {
                incumbent = s;
                incumbent_cost = cost;
            }
        }

        T *= alpha;
    }

    return best;
}