# Solution representation for truck and drone

- Part one: Permutable ordered list of values illustrating the truck route
- Part two: Drone map. Keys for each node (even node 0). Value: List of length *amount of drones*. Each entry is a tuple consisting of (delivery destination, landing point).


This might be enough, or it might be useful to also introduce a map going in the opposite direction. 

Also need to implement some method to convert from this to the wanted solution format (str)

Drone limit feasibility check: Go through all landing keys and their values. For each value, if not empty, check key -> value[0] -> value[1] within limit.

As long as all of other operations keep all places visited consistent, we don't actually need an efficient way to check if all nodes are covered!!!

## Issues: 

Feasibility check may be a little slow.

# scribbles
Operations we want to do:

- permute truck route
Should be fast on truck route itself no matter how we represent. However, we would also have to go through all drones which have either a start point or end point at each of the two nodes that were swapped in order to check if the new solution has a drone now has an illegal solution. If this is the case, swap the start point and end point of the drone (should solve all cases I think)



