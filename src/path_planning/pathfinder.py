#!/usr/bin/env python
import math

import numpy as np

# http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html
import rospy
from tsp import tsp_dp as tsp, create_distance_matrix
from delivery import Delivery

import planner

# Constants
# OCCUPANCY_THRESHOLD = 50  # occupancy probabilities are in the range [0,100].  Unknown is -1.


class PathFinder:
    """Secondary methods to find the shortest path within the OccupancyGrid between the start and goal points using the
        A* algorithm"""

    def __init__(self, grid, goal):
        self.grid = grid
        self.goal_state = goal

    def is_path_valid(self, start):
        """This method checks if the chosen start and goal points are within the designated grid borders
        and/or not in spaces occupied by obstacles"""
        is_valid = True  # path is valid until proven otherwise

        # if the start and/or goal cell is outside the boundaries of the grid, the path is invalid
        if start[0] < 0 or start[0] >= self.grid.width or start[1] < 0 or start[1] >= self.grid.height or \
                self.goal_state[0] < 0 or self.goal_state[0] >= self.grid.width or self.goal_state[1] < 0 or self.goal_state[1] >= self.grid.height:
            is_valid = False

        # if start and/or goal cell is already occupied by an obstacle, the path is invalid
        elif self.grid.is_occupied(start[0], start[1]) or self.grid.is_occupied(self.goal_state[0], self.goal_state[1]):
            # print("start occupied: " + str(self.grid.is_occupied(start[0], start[1])))
            # print("end occupied: " + str(self.grid.is_occupied(self.goal_state[0], self.goal_state[1])))
            # print("is_path_valid: occupied")
            is_valid = False

        return is_valid
    
    def find_closest_valid_cell(self, point):
        # Initialize a queue with the start point
        queue = [point]
        visited = []

        # While there are points to process
        while queue:
            # Pop the first point
            newPoint = queue.pop(0)

            # If this point is not an obstacle
            if not self.grid.is_occupied(newPoint[0], newPoint[1]):
                # If there are free neighbors
                if all(not self.grid.is_occupied(neighbor[0], neighbor[1]) for neighbor in self.get_neighbors(newPoint, x=3, valid=False)):
                    # Return it
                    return newPoint

            # Else, add its neighbors to the queue
            for neighbor in self.get_neighbors(newPoint, valid=False):
                if neighbor not in visited:
                    visited.append(neighbor)
                    queue.append(neighbor)

        # If no points are found, return None
        print("No valid cell found")
        return None
   

    def euclidian_heuristic(self, state):
        """Returns euclidian distance from position in current state and in goal state"""
        x_distance = self.goal_state[0] - state[0]
        y_distance = self.goal_state[1] - state[1]
        return math.sqrt(x_distance ** 2 + y_distance ** 2)

    def get_neighbors(self, state, x=1, valid=True):
        """Finds all valid moves from current state, x steps away. Possible moves are left, right, up, down, and diagonals.

        Representation of grid of neighbors for x = 1:
                (-1,1)  (0,1)   (1,1)
                (-1,0)  <here>  (1,0)
                (-1,-1) (0,-1)  (1,-1)

        For x > 1, it expands this grid proportionately.
        """
        neighbors = []
        # Multiplying directions_to_move by x to get the further neighbors
        directions_to_move = [(x*-1, x*1), (0, x*1), (x*1, x*1), 
                              (x*-1, 0),                (x*1, 0), 
                              (x*-1, x*-1), (0, x*-1), (x*1, x*-1)]

        for direction in directions_to_move:
            new_pos = (state[0] + direction[0], state[1] + direction[1])
            if valid:
              if not self.grid.is_occupied(new_pos[0], new_pos[1]):
                  neighbors.append(new_pos)
            else:
                neighbors.append(new_pos)

        return neighbors


    def get_cost(self, node, child_node):
        """Gets the transition cost between two states. While it is known the cost to move left/right/up/down is 1,
        this accounts for moving diagonally, which has a cost of root 2"""
        return math.sqrt((node[0] - child_node[0]) ** 2 + (node[1] - child_node[1]) ** 2)


def main():
    """main function."""
    rospy.init_node("planner")
    
    plan = planner.Planner()
    
    # manually assigned locations
    dorms = {
        "gile": (10,25.1),
        "mid mass": (14.1 ,21.5),
        "woodward": (25.5,22),
        "andre": (28.7, 20.5),
        "butterfield": (12.3,27.4),
        "topliff": (23.2,17)
      }
      
    restaurants = {
        "tuk tuk": (14.75, 15.8),
        "molly": (15.50, 11.0),
        "han": (18.50, 10.5),
        "sushiya": (16.7, 8.0),
        "center" : (16.5, 16.5)
      }
      
    start = (16, 16)
    
    delivery = Delivery(dorms, restaurants, start)
    path = None
    
    # pick the resturant and dorms
    nameList = ["molly", "tuk tuk", "gile", "woodward"]
    deliveryList = delivery.getPoints(nameList)

    distances = create_distance_matrix(deliveryList, plan.find_path)
    
    optimal_path, optimal_cost = tsp(distances, 0)
    
    path_format = delivery.pathToNames(nameList, optimal_path)
      
    # Print the results
    print("The optimal path is {} with total cost {}".format(path_format, optimal_cost))

    # to display the dorms and resturants on 
    resolution = 0.05
    #  convert points to cells
    r = delivery.pointsToCells(resolution, "restaurants")      
    d = delivery.pointsToCells(resolution, "dorms")   
    
    plan.delete_markers()  # clear markers from before 
    plan.publish_markers(r.values())
    plan.publish_poses(r.values())

    plan.publish_markers(d.values())
    plan.publish_poses(d.values())
    
    # if path found, display path on rviz
    pathInPoints = delivery.pathToPoints(path_format)
    print(pathInPoints)
    for i in range(len(pathInPoints)-1):
      path = plan.find_path(pathInPoints[i], pathInPoints[i+1])
      if path:
        plan.publish_markers(path)
        plan.publish_poses(path)


if __name__ == "__main__":
    """ run the main function. """
    main()
