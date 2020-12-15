# Iterative Deepening A* Search (IDA\*)

## Description
IDA* Search combines the advantages of both depth-first search and 
breadth-first search within a tree to allow searching agents to use the least 
amount of memory and time to find the most optimal path to a goal state. It 
uses a heuristic to find the estimated minimum cost to the goal state and uses 
that as a bound. The agent explores using depth-first search until the bound is 
reached and switches to a new path until a solution is found or all 
possibilities are explored for that bound. If no solution is found, the bound 
is increased and the agent starts over from the root. This solves the issue of 
memory from breadth-first search by only storing nodes within the bound and 
vistsed nodes instead of the entire frontier. It also helps depth-first search 
reduce the time complexity by only expanding the frontier using the minimum 
necessary bound instead of exploring the deepest paths where a solution might 
not even exist.

## Implementation
First, I set the first bound to be the root's manhattan heuristic value. 
This value gets updated in get_action with the next min cut-off f value, which 
was kept track of in the visit method. The program then visits the current node 
to find child nodes to expand. Only nodes that are below the bound are stored 
in the queue of nodes to visit. These nodes are stored in a priority queue so 
the algorithm will expand nodes with the lowest f value first. After visiting 
the node, the program starts off by finding the next actions to take in 
get_action. If there are no actions left to take, then the bound is updated 
to the next min cut-off f value and starts over with new paths to explore. 
This is repeated until the goal has been reached. 

## Instructions
0. Install [OpenNERO](https://github.com/Grem-Lin/Opennero).
1. Open your terminal and clone the repository by typing the following command 
into your terminal: 
	
		git clone https://github.com/kd24972/IterativeDeepeningAStarSearch.git

2. Copy the folder that was just cloned and place it in: 
	
		OpenNERO.app/Contents/Resources

3. Run the new mod in OpenNERO.