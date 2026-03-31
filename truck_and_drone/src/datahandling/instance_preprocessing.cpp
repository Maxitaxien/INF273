#include "datahandling/instance_preprocessing.h"
#include <cstddef>

namespace
{
bool valid_node(const Instance &instance, int node)
{
    return node >= 0 && node < (int)(instance.drone_matrix.size());
}

std::size_t cube_index(
    std::size_t dim,
    int launch_node,
    int customer,
    int land_node)
{
    return ((std::size_t)(launch_node) * dim + (std::size_t)(customer)) * dim +
        (std::size_t)(land_node);
}
}

void precompute_pure_drone_feasibility(Instance &instance)
{
    const std::size_t dim = instance.drone_matrix.size();
    if (dim == 0)
    {
        instance.pure_drone_feasible.clear();
        return;
    }

    instance.pure_drone_feasible.assign(dim * dim * dim, 0);
    for (int launch_node = 0; launch_node < (int)(dim); ++launch_node)
    {
        for (int customer = 0; customer < (int)(dim); ++customer)
        {
            for (int land_node = 0; land_node < (int)(dim); ++land_node)
            {
                const long long pure_duration =
                    instance.drone_matrix[launch_node][customer] +
                    instance.drone_matrix[customer][land_node];
                instance.pure_drone_feasible[cube_index(
                    dim,
                    launch_node,
                    customer,
                    land_node)] = pure_duration <= instance.lim ? 1 : 0;
            }
        }
    }
}

bool pure_drone_flight_within_limit(
    const Instance &instance,
    int launch_node,
    int customer,
    int land_node)
{
    if (!valid_node(instance, launch_node) ||
        !valid_node(instance, customer) ||
        !valid_node(instance, land_node))
    {
        return false;
    }

    const std::size_t dim = instance.drone_matrix.size();
    if (instance.pure_drone_feasible.size() == dim * dim * dim)
    {
        return instance.pure_drone_feasible[cube_index(
                   dim,
                   launch_node,
                   customer,
                   land_node)] != 0;
    }

    const long long pure_duration =
        instance.drone_matrix[launch_node][customer] +
        instance.drone_matrix[customer][land_node];
    return pure_duration <= instance.lim;
}
