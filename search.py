# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
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
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))

    util.raiseNotDefined()
    "*** YOUR CODE HERE ***"
    """
    from util import Stack

    noMovement = []
    # no need to move anywhere
    # if we reached to the goal state!
    if problem.isGoalState(problem.getStartState()):
        return noMovement

    frontier = Stack()
    node = problem.getStartState()
    startingPath = []
    explored = []

    # the search starts right after
    # we are entering the starting state!
    frontier.push((node, startingPath))
    while True:
        # incase there is no path to our goal state!
        if frontier.isEmpty():
            return noMovement

        node, path = frontier.pop()
        # saves all the states that pacman already been in
        explored.append(node)

        # returns the path that pacman did for finding the goal state
        if problem.isGoalState(node):
            return path

        # entering to the frontier all the possible ways that
        # pacman can do from his current state
        for successor in problem.getSuccessors(node):
            child = successor[0]
            childPath = path + [successor[1]]

            if child not in explored:
                frontier.push((child, childPath))


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue
    # no need to move anywhere
    # if we reached to the goal state!
    noMovement = []
    if problem.isGoalState(problem.getStartState()):
        return noMovement

    frontier = Queue()
    startingPath = []
    explored = []

    # the search starts right after
    # we are entering the starting state!
    frontier.push((problem.getStartState(), startingPath))
    while True:
        # incase there is no path to our goal state!
        if frontier.isEmpty():
            return noMovement

        node, path = frontier.pop()
        explored.append(node)

        # returns the path that pacman did for finding the goal state
        if problem.isGoalState(node):
            return path
        # entering to the frontier all the possible ways that
        # pacman can do from his current state
        for successor in problem.getSuccessors(node):
            child = successor[0]
            if child not in explored:
                childPath = path + [successor[1]]
                explored.append(child)
                frontier.push((child, childPath))


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    from util import PriorityQueue
    # no need to move anywhere
    # if we reached to the goal state!
    noMovement = []
    if problem.isGoalState(problem.getStartState()):
        return noMovement

    node = problem.getStartState()
    frontier = PriorityQueue()
    explored = []

    # the search starts right after
    # we are entering the starting state!
    startingPath = []
    frontier.push((node, startingPath), 0)
    while True:
        # incase there is no path to our goal state!
        if frontier.isEmpty():
            return noMovement

        node, path = frontier.pop()
        explored.append(node)

        # returns the path that pacman did for finding the goal state
        if problem.isGoalState(node):
            return path

        for successor in problem.getSuccessors(node):
            child = successor[0]
            childPath = path + [successor[1]]
            childCost = problem.getCostOfActions(childPath)

            # now we need to check first if pacman has been in this state before
            # if yes - we will check if in this current path pacman walked
            # less than the other one.
            childInFrontier = False
            if child not in explored:
                for state in frontier.heap:
                    if child == state[2][0]:
                        childInFrontier = True

                # incase pacman never been there before
                if childInFrontier is False:
                    frontier.push((child, childPath), childCost)
                else:
                    # here we will find the old path that pacman already walked
                    # through and we will keep the path that done minimum actions.
                    for state in frontier.heap:
                        if child == state[2][0]:
                            statePathCost = problem.getCostOfActions(state[2][1])
                            if statePathCost > childCost:
                                frontier.update((child, childPath), childCost)


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueueWithFunction
    # no need to move anywhere
    # if we reached to the goal state!
    noMovement = []
    if problem.isGoalState(problem.getStartState()):
        return noMovement

    frontier = PriorityQueueWithFunction(
        lambda n: problem.getCostOfActions(n[1]) + heuristic(n[0], problem))
    explored = []
    node = problem.getStartState()
    statingPath = []

    # the search starts right after
    # we are entering the starting state!
    frontier.push((node, statingPath, heuristic))
    while True:
        # incase there is no path to our goal state!
        if frontier.isEmpty():
            return noMovement

        node, path, cost = frontier.pop()
        if node in explored:
            while not frontier.isEmpty() and node in explored:
                node, path, cost = frontier.pop()

        explored.append(node)

        # returns the path that pacman did for finding the goal state
        if problem.isGoalState(node):
            return path

        for successor in problem.getSuccessors(node):
            child = successor[0]
            childPath = path + [successor[1]]
            if child not in explored:
                frontier.push((child, childPath, heuristic))


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
