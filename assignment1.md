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
- We can easily permute within final sections to change which daughters cover which ports or potentially remove/introduce daughters


### Drawbacks:
- If we permute in such a way that a daughter no longer covers any ports, we need to also the second part.
- In a similar vein, if we introduce/reintroduce a daughter ship, a start node entry must be added to part 2



