from OpenNero import *
from common import *

import Maze
from Maze.constants import *
import Maze.agent
from Maze.agent import *

from heapq import heappush, heappop

# from Maze/agent.py
def manhattan_heuristic(r, c):
    return abs(ROWS - 1 - r) + abs(COLS - 1 - c)

class IdaStarSearchAgent(SearchAgent):
    """
    IDA* algorithm
    """
    def __init__(self):
        # this line is crucial, otherwise the class is not recognized as an AgentBrainPtr by C++
        SearchAgent.__init__(self)
        # minimize the Manhattan distance
        self.heuristic = manhattan_heuristic
        self.next_bound = sys.maxint # bound's next min
        self.bound = None
        self.reset()

    def reset(self):
        """
        Reset the agent
        """
        self.parents = {}
        self.queue = [] # queue of cells to visit (front)
        self.visited = set([]) # set of nodes we have visited
        self.enqueued = set([]) # set of things in the queue (superset of visited)
        self.backpointers = {} # a dictionary from nodes to their predecessors
        self.starting_pos = None
        self.goal = None # we have no idea where to go at first
        pass

    def initialize(self, init_info):
        """
        Initializes the agent upon reset
        """
        self.action_info = init_info.actions
        return True

    # from Maze/agent.py
    def enqueue(self, cell):
        (r, c) = cell
        d = self.get_distance(r, c)
        h = self.heuristic(r, c)
        print "Queuing cell (%s, %s), d = %s, h = %s" % (r, c, d, h)
        heappush(self.queue, Cell(d + h, r, c))

    # from Maze/agent.py
    def dequeue(self):
        cell = heappop(self.queue)
        h, r, c = cell.h, cell.r, cell.c
        return (r, c)


    def initialize(self, init_info):
        self.constraints = init_info.actions
        return True

    def start(self, time, observations):
        """
        Called on the first move
        """
        r = observations[0]
        c = observations[1]
        self.starting_pos = (r, c)
        get_environment().mark_maze_white(r, c)
        self.starting_pos = (r, c)
        # set the initial bound to root's h val
        self.bound = self.heuristic(r, c)
        self.visit(r, c, observations)
        return self.get_action(r, c, observations)

    # skeleton from Maze/agent.py
    def get_action(self, r, c, observations):
        # check to see if we need to reset bound
        did_reset = False
        if not self.goal: # first, figure out where we are trying to go
            if not self.queue: 
                # if we run out of nodes to expand, increase the bound
                self.goal = (0, 0)
                self.bound = self.next_bound
                self.next_bound = sys.maxint
                self.reset()
                did_reset = True
            else: 
                # expand the next node
                (r2,c2) = self.dequeue()
                get_environment().unmark_maze_agent(r2,c2)
                get_environment().mark_maze_yellow(r2,c2)
                self.goal = (r2, c2)
        # then, check if we can get there
        if (did_reset): 
            r2, c2 = 0, 0
        else: 
            r2, c2 = self.goal
        dr, dc = r2 - r, c2 - c
        action = get_action_index((dr,dc)) # try to find the action (will return None if it's not there)
        v = self.constraints.get_instance() # make the action vector to return
        # first, is the node reachable in one action?
        if action is not None and observations[2 + action] == 0:
            v[0] = action # if yes, do that action!
        else:
            # if not, we should teleport and return null action
            get_environment().teleport(self, r2, c2)
            v[0] = 4
        return v # return the action

    # skeleton from Maze/agent.py
    def visit(self, row, col, observations):
        """
        visit the node row, col and decide where we can go from there
        """
        if (row, col) not in self.visited:
            self.mark_visited(row, col)
        # we are at row, col, so we mark it visited:
        self.visited.add((row, col))
        self.enqueued.add((row, col)) # just in case
        # if we have reached our current subgoal, mark it visited
        if self.goal == (row, col):
            print  'reached goal: ' + str((row, col))
            self.goal = None
        # then we queue up some places to go next
        for i, (dr, dc) in enumerate(MAZE_MOVES):
            if observations[2 + i] == 0: # are we free to perform this action?
                # the action index should correspond to sensor index - 2
                r2 = row + dr # compute the row we could move to
                c2 = col + dc # compute the col we could move to
                dist = self.get_distance(row, col) + 1
                heur = self.heuristic(r2, c2)
                f = dist + heur
                # visit node if it's f val is less than the bound
                if (f <= self.bound): 
                    if (r2, c2) not in self.enqueued:
                        self.mark_the_front(row, col, r2, c2)
                        assert self.backpointers.get((row, col)) != (r2, c2)
                        self.backpointers[(r2, c2)] = row, col # remember where we (would) come from
                        self.enqueued.add((r2, c2))
                        self.enqueue((r2, c2)) 
                else: 
                    # skip this node, but keep track of the min t val
                    if (f < self.next_bound): 
                        self.next_bound = f

    # from Maze/agent.py
    def get_next_step(self, r1, c1, r2, c2):
        """
        return the next step when trying to get from current r1,c1 to target r2,c2
        """
        back2 = [] # cells between origin and r2,c2
        r, c = r2, c2 # back track from target
        while (r, c) in self.backpointers:
            r, c = self.backpointers[(r, c)]
            if (r1, c1) == (r, c): # if we find starting point, we need to move forward
                return back2[-1] # return the last step
            back2.append((r, c))
        return self.backpointers[(r1, c1)]

    # from Maze/agent.py
    def act(self, time, observations, reward):
        """
        Choose an action after receiving the current sensor vector and the instantaneous reward from the previous time step.
        """
        # interpret the observations
        row = int(observations[0])
        col = int(observations[1])
        # first, visit the node we are in and queue up some places to go
        self.visit(row, col, observations)
        # now we have some candidate states and a way to return if we don't like it there, so let's try one!
        return self.get_action(row, col, observations)

    def end(self, time, reward):
        """
        at the end of an episode, the environment tells us the final reward
        """
        print  "Final reward: %f, cumulative: %f" % (reward[0], self.fitness[0])
        self.reset()
        return True

    def destroy(self):
        """
        After one or more episodes, this agent can be disposed of
        """
        return True

    # from Maze/agent.py
    def mark_the_front(self, r, c, r2, c2):
        get_environment().mark_maze_green(r2, c2)

    # from Maze/agent.py
    def mark_target(self, r, c):
        get_environment().mark_maze_yellow(r, c)

    # from Maze/agent.py
    def mark_visited(self, r, c):
        if (r, c) != self.starting_pos:
            get_environment().mark_maze_blue(r, c)

    # from Maze/agent.py
    def mark_path(self, r, c):
        get_environment().mark_maze_white(r, c)
