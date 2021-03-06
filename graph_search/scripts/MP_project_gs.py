#!/usr/bin/env python
'''
Package providing helper classes and functions for performing graph search operations for planning.
'''
import sys
import numpy as np
import heapq
import matplotlib.pyplot as plotter
from math import hypot, sqrt

_DEBUG = False
_DEBUG_END = True
_ACTIONS = ['u','d','l','r']
#_ACTIONS = ['r','l','d','u'] #reversed action set

_ACTIONS_2 = ['u','d','l','r','ne','nw','sw','se']
_ACTIONS_3 = ['2df','2dl','2dr'] #foreward, left, right

_T = 2 #Theta
_X = 1
_Y = 0
_GOAL_COLOR = 0.75
_INIT_COLOR = 0.25
_PATH_COLOR_RANGE = _GOAL_COLOR-_INIT_COLOR
_VISITED_COLOR = 0.9


class GridMap:
    '''
    Class to hold a grid map for navigation. Reads in a map.txt file of the format
    0 - free cell, x - occupied cell, g - goal location, i - initial location.
    Additionally provides a simple transition model for grid maps and a convience function
    for displaying maps.
    '''
    def __init__(self, start=(None,None), goal=(None,None), map_path= None):
        '''
        Constructor. Makes the necessary class variables. Optionally reads in a provided map
        file given by map_path.

        map_path (optional) - a string of the path to the file on disk
        '''
        self.rows = None
        self.cols = None
        self.goal = goal
        self.init_pos = start
        #print(type(map_path))
        if type(map_path) == np.ndarray:
            #print('yes!')
            self.rows = len(map_path)
            self.cols = len(map_path[0])
            self.IG_grid = np.zeros((self.rows,self.cols))
            for r in range(self.rows):
                for c in range(self.cols):
                    self.IG_grid[r][c] = 1/(float(map_path[r][c])+1) #cost
                    #self.IG_grid[r][c] = float(lines[r][c]) #avg_cost

            #print(self.IG_grid)
        elif type(map_path) == str:
            self.read_map(map_path)



    def read_map(self, map_path):
        '''
        Read in a specified map file of the format ....

        map_path - a string of the path to the file on disk
        '''
        map_file = open(map_path,'r')
        lines = [l.rstrip().lower() for l in map_file.readlines()]
        #print('test')
        #print(lines)
        map_file.close()

        self.rows = len(lines)
        self.cols = max([len(l) for l in lines])
        self.IG_grid = np.zeros((self.rows,self.cols))

        if _DEBUG:
            print('rows', self.rows)
            print('cols', self.cols)
            print(lines)



        #create grid

        #read the map as a numpy table
        for r in range(self.rows):
            for c in range(self.cols):

                self.IG_grid[r][c] = 1/(float(lines[r][c])+1) #cost
                #self.IG_grid[r][c] = float(lines[r][c]) #avg_cost

        self.init_pos = (r,c,0)
        #print(self.IG_grid)
        #print(type(self.IG_grid[1][1]))

        #Find goal function
        self.goal = (1,1)

    def is_goal(self,s):
        '''
        Test if a specifid state is the goal state

        s - tuple describing the state as (row, col) position on the grid.

        Returns - True if s is the goal. False otherwise.
        '''
        return (s[_X] == self.goal[_X] and
                s[_Y] == self.goal[_Y])

    def transition(self, s, a):
        '''
        Transition function for the current grid map.

        s - tuple describing the state as (row, col) position on the grid.
        a - the action to be performed from state s

        returns - s_prime, the state transitioned to by taking action a in state s.
        If the action is not valid (e.g. moves off the grid or into an obstacle)
        returns the current state.
        '''
        new_pos = list(s[:])
        # Ensure action stays on the board

        if a == 'u':
            if s[_Y] > 0:
                new_pos[_Y] -= 1
        elif a == 'd':
            if s[_Y] < self.rows - 1:
                new_pos[_Y] += 1
        elif a == 'l':
            if s[_X] > 0:
                new_pos[_X] -= 1
        elif a == 'r':
            if s[_X] < self.cols - 1:
                new_pos[_X] += 1
        elif a == 'se':
            if s[_Y] < self.rows - 1 and s[_X] < self.cols - 1:
                new_pos[_Y] +=1
                new_pos[_X] +=1
        elif a == 'ne':
            if s[_Y] > 0 and s[_X] < self.cols - 1:
                new_pos[_Y] -=1
                new_pos[_X] +=1
        elif a == 'sw':
            if s[_Y] < self.rows - 1 and s[_X] > 0:
                new_pos[_Y] +=1
                new_pos[_X] -=1
        elif a == 'nw':
            if s[_Y] >  0 and s[_X] > 0:
                new_pos[_Y] -=1
                new_pos[_X] -=1

        else:
            print('Unknown action:', str(a))


        s_prime = tuple(new_pos)
        #print(len(self.IG_grid))
        #print(len(self.IG_grid[0]))

        #print('x',new_pos[_X])
        #print('y',new_pos[_Y])

        IG = self.IG_grid[new_pos[_Y],new_pos[_X]]

        return s_prime, IG

    def display_map(self, path=[], visited={}, filename=None):
        '''
        Visualize the map read in. Optionally display the resulting plan and visisted nodes

        path - a list of tuples describing the path take from init to goal
        visited - a set of tuples describing the states visited during a search
        filename - relative path to file where image will be saved
        '''
        display_grid = np.array(self.IG_grid, dtype=np.float32)
        if path is None:
            print('There is no Path!')

        if path is not None:
            # Color all visited nodes if requested
            for v in visited:
                display_grid[v[0],v[1]] = _VISITED_COLOR
            # Color path in increasing color from init to goal
            for i, p in enumerate(path):
                disp_col = _INIT_COLOR + _PATH_COLOR_RANGE*(i+1)/len(path)
                display_grid[p[1],p[0]] = disp_col

        display_grid[self.init_pos[0],self.init_pos[1]] = _INIT_COLOR
        display_grid[self.goal] = _GOAL_COLOR

        # Plot display grid for visualization
        imgplot = plotter.imshow(display_grid)
        # Set interpolation to nearest to create sharp boundaries
        imgplot.set_interpolation('nearest')
        # Set color map to diverging style for contrast
        imgplot.set_cmap('Spectral')
        if filename is not None:
            plotter.savefig(filename)
        plotter.show()

    def manhatten_heuristic(self, s):
        '''
        Example of how a heuristic may be provided. This one is admissable, but dumb.

        s - tuple describing the state as (row, col) position on the grid.

        returns - floating point estimate of the cost to the goal from state s
        '''
        goal = self.goal

        dist = manhat_dist(goal, s)

        return dist
    def euclidean_heuristic(self, s):
        goal = self.goal

        dist = eucl_dist(goal, s)

        return dist


class SearchNode:
    def __init__(self, s, A, parent=None, parent_action=None, cost=0, IG=0):
        '''
        s - the state defining the search node
        A - list of actions
        parent - the parent search node
        parent_action - the action taken from parent to get to s
        '''
        self.parent = parent
        self.cost = cost
        self.parent_action = parent_action
        self.state = s[:]
        self.actions = A[:]
        self.IG = IG

    def __str__(self):
        '''
        Return a human readable description of the node
        '''
        return str(self.state) + ' ' + str(self.actions)+' '+str(self.parent)+' '+str(self.parent_action)
    def __lt__(self, other):
        return self.cost < other.cost

class PriorityQ:
    '''
    Priority queue implementation with quick access for membership testing
    Setup currently to only with the SearchNode class
    '''
    def __init__(self):
        '''
        Initialize an empty priority queue
        '''
        self.l = [] # list storing the priority q
        self.s = set() # set for fast membership testing

    def __contains__(self, x):
        '''
        Test if x is in the queue
        '''
        return x in self.s

    def push(self, x, cost):
        '''
        Adds an element to the priority queue.
        If the state already exists, we update the cost
        '''
        if x.state in self.s:
            for i in self.l:
                if x.state == i[1].state:
                    if cost < i[0]:
                        return self.replace(x, cost)
                        #return self.replace(x, i[0]) #do nothing
                    #else:

        if x.state not in self.s:
            heapq.heappush(self.l, (cost, x))
            self.s.add(x.state)

    def pop(self,idx=None):
        '''
        Get the value and remove the lowest cost element from the queue
        '''

        if idx != None:
            x = self.l.pop(idx)
            self.s.remove(x[1].state)
            return x[0],x[1]
        x = heapq.heappop(self.l)
        self.s.remove(x[1].state)
        return x[0],x[1]

    def peak(self):
        '''
        Get the value of the lowest cost element in the priority queue
        '''
        x = self.l[0]
        return x[1]

    def __len__(self):
        '''
        Return the number of elements in the queue
        '''
        return len(self.l)

    def replace(self, x, new_cost):
        '''
        Removes element x from the q and replaces it with x with the new_cost
        '''
        for y in self.l:
            if x.state == y[1].state:
                self.l.remove(y)
                self.s.remove(y[1].state)
                break
        heapq.heapify(self.l)
        self.push(x, new_cost)

    def get_cost(self, x):
        '''
        Return the cost for the search node with state x.state
        '''
        for y in self.l:
            if x.state == y[1].state:
                return y[0]

    def __str__(self):
        '''
        Return a string of the contents of the list
        '''
        return str(self.l)

def RRT_search(current_state, IG_map):
    '''
    The idea is to for every step, load the map and step in the direction with highest information gain.
    Then update particle filter, then make a new step.
    '''



def IG_search(init_state, f, is_goal, actions):
    '''
    init_state - value of the initial state
    f - transition function takes input state (s), action (a), returns s_prime = f(s, a)
        returns s if action is not valid
    is_goal - takes state as input returns true if it is a goal state
        actions - list of actions available
    actions - set of actions which can be taken by the agent

    returns - ((path, action_path), visited) or None if no path can be found
    path - a list of tuples. The first element is the initial state followed by all states
        traversed until the final goal state
    action_path - the actions taken to transition from the initial state to goal state
    '''
    frontier = PriorityQ()
    #action_path = []
    cummulative_cost = 0.0
    n0 = SearchNode(init_state, actions)
    visited = {} #Initializing list of visited nodes
    frontier.push(n0,cummulative_cost)
    while len(frontier) > 0:

        #print('queue')

        #for node in frontier.l:
            #print(node[1].state, node[0])
        cost,n_i = frontier.pop()
        #cost = cost

        #print('\n',n_i.state, 'got popped with cost',cost)

        if n_i.state in visited.keys():
            if visited[n_i.state] < cost:

                continue

        if n_i.state not in visited:
            visited[n_i.state] = cost


        if is_goal(n_i.state):
                #Backtrack to find the shortest path

            return (backpath(n_i), visited.keys())
        else:
            for a in actions:

                s_prime,IG = f(n_i.state,a)
                #print('IGS',IG)
                n_prime = SearchNode(s_prime,actions,n_i,a,0,IG)
                #print(n_prime.IG)
                if n_prime.state in visited.keys():
                    continue
                frontier.push(n_prime,costfunction(cost, n_prime)) #function not defined


    return ((None,None),None)


def backpath(node):
    '''
    Function to determine the path that lead to the specified search node

    node - the SearchNode that is the end of the path

    returns - a tuple containing (path, action_path) which are lists respectively of the states
    visited from init to goal (inclusive) and the actions taken to make those transitions.
    '''
    path = []
    path.append(node.state)
    action_path = []
    while node.parent != None:
        path.append(node.parent.state)
        action_path.append(node.parent_action)
        node = node.parent

    path = list(reversed(path))
    action_path = list(reversed(action_path))

    #print('path',path)
    path2 = []
    for i in path:

        step = np.array([i[1]*2+1,(10-i[0])*10-5])
        path2.append(step)

    path2 = np.asarray(path2)
    #print('path2',path2)
    #print(type(path2[0]))



    return (path2, action_path)


def manhat_dist(tuple1,tuple2):
    #Manhatten distance between two tuples
    return abs(abs(tuple1[0]-tuple2[0])+abs(tuple1[1]-tuple2[1]))

def eucl_dist(tuple1,tuple2):
    #Finding the eucleadian distance between two positions.

    return sqrt((tuple1[0]-tuple2[0])**2 + (tuple1[1]-tuple2[1])**2)

def costfunction(old_cost, new_node):
    #print('cf',new_node.IG)
    cost = old_cost + new_node.IG

    return cost
def avg_costfunction(old_cost, new_node):
    #print('cf',new_node.IG)
    i=0
    cummulative_cost = new_node.IG
    parent = new_node.parent
    while parent is not None:
        cummulative_cost = cummulative_cost + parent.IG
        parent = parent.parent

        i = i+1

    cost = -(cummulative_cost/i)

    return cost
