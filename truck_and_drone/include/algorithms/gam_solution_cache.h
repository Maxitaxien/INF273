#pragma once

#include <string>
#include <unordered_map>

struct Instance;
struct Solution;

enum class GAMFeasibilityMode
{
    AssumeFeasible,
    VerifyWithMasterCheck,
};

struct GAMCachedSolution
{
    bool feasible_known = false;
    bool feasible = false;
    bool objective_known = false;
    long long objective = 0;
};

using GAMSolutionCache = std::unordered_map<std::string, GAMCachedSolution>;

struct GAMSolutionEvaluation
{
    bool is_new_solution = false;
    bool feasible = false;
    bool objective_known = false;
    long long objective = 0;
};

std::string gam_solution_cache_key(const Solution &solution);

GAMCachedSolution &gam_solution_cache_entry(
    GAMSolutionCache &cache,
    const Solution &solution,
    bool *is_new_solution = nullptr);

void gam_cache_known_feasible_solution(
    GAMSolutionCache &cache,
    const Instance &instance,
    const Solution &solution,
    long long objective);

GAMSolutionEvaluation evaluate_solution_with_cache(
    const Instance &instance,
    const Solution &solution,
    GAMSolutionCache *cache,
    GAMFeasibilityMode feasibility_mode = GAMFeasibilityMode::AssumeFeasible);
