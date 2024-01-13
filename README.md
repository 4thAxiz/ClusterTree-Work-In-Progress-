# ClusterTree-Work-In-Progress-
## Linearized Binary Spatial-Partitioning Structure (unlike traditional trees that have a O(n log n) overhead) for sub-linear queries (currently unbalanced)
## What is this new novel Cluster Tree data structure?
A new spatial binary data structure, similiar to the idea of kd-trees or quad/oct trees for logarithimic queries, except guranteed linear construction times.
Unlike most traditional trees, ClusterTree only necessarily subdivides cells only for arbitrarily high-density clustered regions, and no divisions for sparse regions. 

### Caveats
- Tree is Unbalanced as of now.
- Static-tree, not intended as a dynamic structure yet.
- Sub-divisions are static as well. Dynamic adaptive sub-division is to come, when balancing mechanism implementation are done. As of now, you must choose a suitable ``subdivision`` value...
 
Experimental, work in progress.
Research paper coming soon
Visualization: https://gyazo.com/46ad45553879e1377b2064dba04b6fc3.mp4
