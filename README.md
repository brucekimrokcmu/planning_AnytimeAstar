# planning

This work is to implement a planner where a point robot intercepts a moving target in a 2D grid map. 
The robot can move in 8-connected grid, and each cell has a cost value. 

To catch the moving target, I first implemented a look up table of backward Dijkstra to calculate g* value of all cells to the closest goal in the goal trajectory. Referring to the look up table for my heuristics, I implemented AnytimeAStar search to return the best plan possible within T msecs.

The data structure I used for open list is prioirty queue that sorts by lower F-values and unordered_map for closed list, vistied list, and heuristics map.
