# Assignment 1 INF273

*Design solution representation for Ship Problem*

## Initial idea:
Using the value -1 as separators:
$\mathcal{S} =$ [*nodes visited by mother ship, hub not included*, -1, *for each daughter ship used: startnode, ordered list of all nodes visited, -1 as separator between daughter ships*]

### Benefits:
- Would be a 1-1 mapping
- Clearly separates mother ship and daughter ships

### Drawbacks:
- No apparent easy way to make simple changes/swaps
- Some complexity in checking validity: The nodes the daughter ship routes start on should be present in mother route, but the internal ones should not.


## Improvement idea:
What if we use something similar, but we treat the start nodes for the daughter routes separately than the internal visited nodes? This might help our validity-checking complexity issues. So something like:
Using the value -1 as separators:
$\mathcal{S} =$ [*nodes visited by mother ship, hub not included*, -1, *list of start nodes for each daughter ship (length of this should be the same as the amount of daughter ships used, and should be a subset of first list)*, -1, *for each daughter ship used: ordered list of all nodes visited (not including start node), -1 as separator between daughter ships*]

### Benefits:
- Same as previous
- Very easy to find number of ships used
- Easier validity checking - between the first part and the part from num 3 onwards, all nodes should appear once.
- We can easily permute within final sections to change which daughters cover which ports or potentially remove/introduce daughters. We can also swap between first section and 3 onwards easily (unless there is a -1)


### Drawbacks:
- If we permute in such a way that a daughter no longer covers any ports, we need to also the second part.
- In a similar vein, if we introduce/reintroduce a daughter ship, a start node entry must be added to part 2
- The order the daughter ships appear in doesn't matter - so we have an uneccesarily large solution space. For instance, if we swap the order the daughter ships appear in, we can make the "same" solution 

### Sample problem representation:

$\mathcal{S} = [6, 4, -1, 6, 4, 6, -1, 5, -1, 7, 8, -1, 3, 2, 1]$

## Yet another improvement idea:

Separate lists? If it is valid.

We essentially want the following properties:

- Mother ship route: List of numbers. Should be permuteable, like a TSP
- Daughter ship hubs: List of numbers which is a subset of mother ship routes. Importantly, we can have duplicates. These do not have to be permuteable.
- Individual routes for daughter ships. List of numbers, should be permutable. The ordering of the lists themselves do not matter.

Rules: 
- Every daughter ship hub should be present in the mother ship routes list.
- The numbers corresponding to ports should appear once and only once in the union of the mother ship route and all daughter ship routes

Proposal for sample problem:

$\mathcal{M} = [6, 4]$ (Permutable ordered list of mother ship hubs.)

$\mathcal{H} = [2, 1]$ (How many daughter ship routes start in mother hub $i$, e.g. here two routes start at $6$ and one at $4$.)

Define $m = sum(\mathcal{H})$. Define $n = \text{amnt of ships}$

There will be $m$ lists of daughter ship routes $\mathcal{D}_m$, each of variable but at most $n - 1$ length.

Here:
$\mathcal{D}_1 = [5]$
$\mathcal{D}_2 = [7, 8]$
$\mathcal{D}_3 = [3, 2, 1]$

### Drawbacks:

- There is no longer any direct linkage showing which daughter ships are located at each hubs.
    - This could possibly be solved by having a mapping? Such as replacing $\mathcal{H}$ with: $\mathcal{H}^* = \{6: [1, 2], 4: [3]\}$



## Final representation:

We will have possible daughter ship usages defined as the range: [1, n]. Each of these represents a unique "key" for a daughter ship usage.

$\mathcal{M}$ is an ordered list of ports visited by the mother ship.

$\mathcal{H}$ is a mapping where the keys are the ports visited by the mother ship and the values is an unordered set of daughter ship routes linked to this port.

$\mathcal{D}_i \quad \forall i \in [1, n]$ are individual ordered lists for each daughter ship route. They will be empty for any that are not used.

We still have a little bit of redundancy, as if multiple daughter ships have their hub in the same port, an equivalent solution would exist where their indices are swapped.


### Representation of sample problem using this notation
$\mathcal{M} = [6, 4]$

$\mathcal{H} = \{6: \{1, 2\}, 4: \{3\}, 0: \{4, 5, 6, 7\}$

$\mathcal{D}_1 = [5], \mathcal{D}_2 = [7, 8], \mathcal{D}_3 = [3, 2, 1], \mathcal{D}_4 = \mathcal{D}_5 = \mathcal{D}_6 = \mathcal{D}_7 = []$

### Potential operations:
