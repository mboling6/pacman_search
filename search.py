# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for 
# educational purposes provided that (1) you do not distribute or publish 
# solutions, (2) you retain this notice, and (3) you provide clear 
# attribution to UC Berkeley, including a link to 
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero 
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and 
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    #create open list as a Stack since DFS
    open = util.Stack()
    #create closed list as a set since it is the visited nodes and doesn't need to be changed
    closed = set([])
    #initialize current as the start state and an empty actions list, don't necessarily need costs
    current = (problem.getStartState(), [])
    #add init state to the open list
    open.push(current)

    while open:
        #pop the first node off the open list
        currentNode = open.pop()
        #store the first nodes state
        state = currentNode[0]
        #store the first nodes actions
        actions = currentNode[1]
        #check if goal has been reached and return the list of actions if it has, this is the path/plan
        if problem.isGoalState(state):
            return actions
        #if current nodes state is not in the closed list, add it
        if state not in closed:
            closed.add(state)
            #get the successors that are not in the closed list
            for s in problem.getSuccessors(state):
                #store the state of a successor
                successorState = s[0]
                #store the action of a successor
                successorActions = s[1]
                #if successor has not been visited add its action to the actions list, push the successor's state and actions to the open list
                if not successorState in closed:
                    successorActions = actions + [successorActions]
                    open.push((successorState, successorActions))

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    #create open list as a Queue since BFS
    open = util.Queue()
    #create closed list as a set since it is the visited nodes and doesn't need to be changed
    closed = set([])
    #initialize current as the start state and an empty actions list, don't necessarily need costs
    current = (problem.getStartState(), [])
    #add init state to the open list
    open.push(current)

    while open:
        #pop the first node off the open list
        currentNode = open.pop()
        #store the first nodes state
        state = currentNode[0]
        #store the first nodes actions
        actions = currentNode[1]
        #check if goal has been reached and return the list of actions if it has, this is the path/plan
        if problem.isGoalState(state):
            return actions
        #if current nodes state is not in the closed list, add it
        if state not in closed:
            closed.add(state)
            #get the successors that are not in the closed list
            for s in problem.getSuccessors(state):
                #store the state of a successor
                successorState = s[0]
                #store the action of a successor
                successorActions = s[1]
                #if successor has not been visited add its action to the actions list, push the successor's state and actions to the open list
                if not successorState in closed:
                    successorActions = actions + [successorActions]
                    open.push((successorState, successorActions))


def uniformCostSearch(problem):
    """
    Search the node of least total cost first.
    """
    "*** YOUR CODE HERE *** - should be similar to above just need to take into account cost when choosing which successor to visit first"
    #create open list as a PriorityQueue since UCS
    open = util.PriorityQueue()
    #create closed list as a set since it is the visited nodes and doesn't need to be changed
    closed = set([])
    #initialize current as the start state and an empty actions list and initial cost of 0
    current = (problem.getStartState(), [], 0)
    #add init state to the open list - add cost, must continue to update from current (new cost)
    open.push(current, current[2])

    while open:
        #pop the first node off the open list
        currentNode = open.pop()
        #store the first nodes state
        state = currentNode[0]
        #store the first nodes actions
        actions = currentNode[1]
        #check if goal has been reached and return the list of actions if it has, this is the path/plan
        cost = currentNode[2]
        #if current nodes state is not in the closed list, add it
        if problem.isGoalState(state):
            return actions
        if state not in closed:
            closed.add(state)
            #get the successors that are not in the closed list
            for s in problem.getSuccessors(state):
                #store the state of a successor
                successorState = s[0]
                #store the action of a successor
                successorActions = s[1]
                #store the cost of successor
                successorCost = s[2]
                #if successor has not been visited add its action to the actions list, push the successor's state and actions to the open list
                if not successorState in closed:                    
                    successorActions = actions + [successorActions]
                    successorCost = cost + successorCost
                    next = (successorState, successorActions, successorCost)
                    open.push(next, next[2])


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """
    #create open list as a PriorityQueue since aStar
    open = util.PriorityQueue()
    #create closed list as a set since it is the visited nodes and doesn't need to be changed
    closed = set([])
    #initialize current as the start state and an empty actions list and initial cost of 0, initialize heuristic estimate
    current = (problem.getStartState(), [], 0, 0)
    #add init state to the open list - add cost, must continue to update from current (new cost), update heuristic
    open.push(current, current[2])
    while open: 
        #pop the first node off the open list
        currentNode = open.pop()
        #store the first nodes state
        state = currentNode[0]
        #store the first nodes actions
        actions = currentNode[1]
        #check if goal has been reached and return the list of actions if it has, this is the path/plan
        cost = currentNode[2]
        if problem.isGoalState(state):
            return actions
        #if current nodes state is not in the closed list, add it
        if state not in closed:
            closed.add(state)
            #get the successors that are not in the closed list
            for s in problem.getSuccessors(state):
                #store the state of a successor
                successorState = s[0]
                #store the action of a successor
                successorActions = s[1]
                #store the cost of successor
                successorCost = s[2]
                #if successor has not been visited add its action to the actions list, push the successor's state and actions to the open list
                if not successorState in closed:  
                    successorActions = actions + [successorActions]
                    successorCost = cost + successorCost
                    next = (successorState, successorActions, successorCost, successorCost + heuristic(successorState, problem))
                    open.push(next, next[3]) 


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
