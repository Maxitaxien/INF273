# Assignment 1 INF273

## Liner shipping solution representaiton


To work with this solution representation, we will initialize $[1, n]$ possible daughter ship indices (let $n$ represent the number of ports), as $n$ is the maximum amount of daughter ships that we could possibly want to use.   

The solution representation consists of three elements:
- An ordered list $\mathcal{M}$, consisting of the hubs visited by the mother ship. The continental main port could be added to the front and back of this list, but is here left implicit.
- A mapping $\mathcal{H}_i$ where the keys are the ports visited by the mother ship and the values is an unordered set of daughter ship indices which are linked to this port. For any daughters not in use, they will be stored under the dummy key $0$.
- Ordered lists $\mathcal{D}_i$, representing the ports contained in daughter ship $i$'s circuit. They will be initialized as empty lists for any daughter ships not in use.

As an example, this solution representation would show the instance given in page 24 of the slides as:

$\mathcal{M} = [6, 4]$

$\mathcal{H} = \{6: \{1, 2\}, 4: \{3\}, 0: \{4, 5, 6, 7\} \}$

$\mathcal{D}_1 = [5], \mathcal{D}_2 = [7, 8], \mathcal{D}_3 = [3, 2, 1], \mathcal{D}_4 = \mathcal{D}_5 = \mathcal{D}_6 = \mathcal{D}_7 = []$

Meaning that the mother ship visits first port 6, then port 4, then goes back to the continental port. 

We have three daughter ships in use, as we can see from the amount of non-empty $\mathcal{D}_i$. Daughters 1, and 2 have a hub in port 6, while daughter 3 has a hub in port 4 as we see from $\mathcal{H}$. Examining the entries of the $\mathcal{D}_i$ in use give us the routes of the different daughter ships. For instance, daughter two has its hub in 6 $(2 \in \mathcal{H}[6])$, and moves through nodes 7 and 8 $(\mathcal{D}_2 = [7, 8]$) before returning to 6 to start its route again.

### Positives

The solution representation is capable of presenting any valid solution of the problem. In an attempt to argue this, consider the two extreme cases of the mother ship covering all ports, and the mother ship only covering one port while all other daughters are utilized. 
For the first case, simply: $\mathcal{M} =$ any permutation of all ports, while all other data structures are left empty. 
For the second, let $\mathcal{M} =$ any one single port, say $j$. Then, $\mathcal{H}[j] = \{1 \to n\}$, and every $\mathcal{D}_i$ will have exactly one entry which will be the port they visit before coming back to $j$.

In addition, this will work as a 1-to-1 mapping between solution representation and problem, as the ordering of elements is preserved where necessary (within mother ship route and individual daughter ship routes). 

Using these separate data structures makes some of the operations we would want to apply to the problem easy to implement while maintaining validity. Here are some examples:


- To remove a port $j$ from the mother ship: Remove $j$ from $\mathcal{M}$. Delete key $j$ from $\mathcal{H}$, and move all values $v$ to key 0. For all values $v$, go through their $\mathcal{D}_v$ entries and collect these in a list $L$. Take entries in $L$ and distribute them randomly among other non-empty $\mathcal{D_i}$. Complexity $O(n)$. 
- Permutation within mother route or daughter route: Swap values within the relevant list. Complexity $O(1)$.
- Remove a port from a daughter ship $j$: Take the port and place it either into another daughter ship route or at the end of the mother ship route. If the daughter ship route is now empty, search through all values in $\mathcal{H}$ and move $j$ to key $0$. Complexity $O(n)$, but usually $O(1)$ if daughter ship route is non-empty after removal.

Keeping the mother ship route and daughter ship routes separate in this way will also facilitate eventual feasibility and objective value calculations.

### Drawbacks

The usage of three different kinds of data structures makes the solution representation verbose, decreasing readability. Although this harms its quality, I feel that the computational benefits we gain is some compensation.

In addition, the space introduces a little bit of redundancy, as if multiple daughter ships have their hub in the same port, an equivalent solution would exist where their indices are swapped. However, this is only true if the daughter ships properties are the same across all daughters.

