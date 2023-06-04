from node import Node
from heapq import heappush, heappop
from pathfinder import PathFinder

def find_path(self, start, goal):
   """Assuming the start and goal points create a valid path, this method implements the A* algorithm (based off
   BFS pseudocode from lec11 slides) to find the path with the smallest cost between the two points.


       Knowns (according to class notes):
           priority is calculated as cost(node) + heuristic(node)
           cost(node) = length of path from start to node
           heuristic(node) = euclidian distance from node to goal
    """
   # convert start and goal points into cell format so in the proper reference frame
   start_cell = self.grid.point_to_cell(start[0], start[1])
   goal_cell = self.grid.point_to_cell(goal[0], goal[1])


   path_finder = PathFinder(self.grid, goal_cell)


   # frontier = new queue
   frontier = []
   # pack start_state into a node
   start_node = Node(start_cell, path_finder.euclidian_heuristic(start_cell), 0.0)
   # add node to frontier
   heappush(frontier, start_node)


   # explored = new set, and add start_state to explored
   explored = {start_node.state: 0}


   # assuming the user inputted path is valid,
   # while frontier is not empty,
   if path_finder.is_path_valid(start_cell):
       while frontier:
           # get current_node from the frontier
           current_node = heappop(frontier)
           # get current_state from current_node
           current_state = current_node.state


           # if current_state is the goal:
           if current_state == path_finder.goal_state:
               # backchain from current_node and return solution
               return self.backchain(current_node)
           else:
               # otherwise, for each child of current_state:
               for child_state in path_finder.get_neighbors(current_state):
                   child_cost = current_node.cost + path_finder.get_cost(current_state, child_state)
                   # pack child state into a node, with backpointer to current_node
                   child_node = Node(child_state, path_finder.euclidian_heuristic(child_state), child_cost, current_node)


                   # if child has not been explored,
                   if child_state not in explored:
                       # add the node to the frontier
                       heappush(frontier, child_node)
                       # add child to explored
                       explored[child_state] = child_cost
                   elif child_cost < explored[child_state]:
                       heappush(frontier, child_node)
                       explored[child_state] = child_cost


def backchain(self, node):
   """This method backchains through parents of each A* node to get final path"""
   result = []
   current = node
   while current:
       result.append(current.state)
       current = current.parent


   # currently in the opposite order, so need to reverse
   result.reverse()
   return result



