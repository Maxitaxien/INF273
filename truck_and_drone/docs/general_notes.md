possible that the launch -> deliver -> land are not in the correct sorted order - affects some operators! For instance, in the maximum lookahead check, it may be the case that the launch -> deliver -> land is not sorted.

greedy insert as one operation is allowed - you can abuse this to perform efficient iteration steps

the value of replacing with drone likely increases as the solution intensifies. this is because in the initial solution, it is probably quite bad, making it very unlikely that it is even possible to cover something with a drone because of the long distances

we can capitalize on this by only performing this only when the algorithm has already run for a little bit.

note that it might be decent to just have some kind of shuffling/random operator to do exploration
