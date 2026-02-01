Notes from gpt on swap-and-pop - messes up order but increases efficiency. Maybe worth checking out?

Vector might still be best

Even though vector has O(n) deletion from the middle, in practice:

Modern CPU caches make vector very fast.

If your permutations involve small moves (e.g., removing one element at a time), the cost is usually acceptable.

Swapping two elements in a vector is O(1) — exactly what you need.

Insertion at the end: amortized O(1).

💡 Trick: if you need frequent deletions but can tolerate unordered sequence, you can do “swap-and-pop”:

std::vector<int> route = {0,1,2,3,4};
int remove_index = 2; // remove '2'

// Swap with last element and pop
std::swap(route[remove_index], route.back());
route.pop_back();

Deletion becomes O(1), but the order changes.

For many TSP heuristics, order is often recomputed anyway, so this is very useful.
