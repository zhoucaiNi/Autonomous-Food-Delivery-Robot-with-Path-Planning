#!/usr/bin/env python

import math

import numpy as np
import tf
import time

# http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from heapq import heappush, heappop

# Constants
OCCUPANCY_THRESHOLD = 50  # occupancy probabilities are in the range [0,100].  Unknown is -1.


"""""""Pathfinding Functions"""""""""
class Grid:
    """Adapted from the Grid example code provided in class.
         Converts an OccupancyGrid into a 2D matrix. Encapsulates all functions pertaining to the grid itself"""
    def __init__(self, grid_data, width, height, resolution):
        self.grid = np.reshape(grid_data, (height, width))

        self.width = width
        self.height = height
        self.resolution = resolution

    def cell_at(self, x, y):
        """Returns value of cell at x,y"""
        return self.grid[y, x]

    def cell_to_point(self, col, row):
        """Given the row and column location within the grid, return the corresponding location in the map reference frame"""
        x = self.resolution * col  # multiply cell size by number of columns in the grid, then add to origin
        y = self.resolution * row  # multiply cell size by number of rows in the grid, then add to origin
        p = [x, y]

        return p

    def point_to_cell(self, x, y):
        """Given the current x and y position within the grid, return the corresponding cell (row and col) values"""
        pos_x = int(x / self.resolution)
        pos_y = int(y / self.resolution)
        cell = (pos_x, pos_y)

        return cell

    def is_occupied(self, col, row):
        """This method determines if a specified cell is occupied or not. True if occupied, false if not"""
        return self.cell_at(col, row) > OCCUPANCY_THRESHOLD


class Node:
    """Represents each point in the OccupancyGrid--used in the A* algorithm implementation"""

    def __init__(self, state, heuristic, cost, parent=None):
        self.state = state
        self.heuristic = heuristic
        self.cost = cost
        self.parent = parent

    def priority(self):
        return self.heuristic + self.cost  # definition of priority in this case (from class slides)

    def __lt__(self, other):
        return self.priority() < other.priority()


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
            is_valid = False

        return is_valid

    def euclidian_heuristic(self, state):
        """Returns euclidian distance from position in current state and in goal state"""
        x_distance = self.goal_state[0] - state[0]
        y_distance = self.goal_state[1] - state[1]
        return math.sqrt(x_distance ** 2 + y_distance ** 2)

    def get_neighbors(self, state):
        """Finds all valid moves from current state. Possible moves are left, right, up, down, and diagonals.

        Representation of grid of neighbors:
                (-1,1)  (0,1)   (1,1)
                (-1,0)  <here>  (1,0)
                (-1,-1) (0,-1)  (1,-1)
        """
        neighbors = []
        directions_to_move = [(-1, 1), (0, 1), (1, 1), (-1, 0), (1, 0), (-1, -1), (0, -1), (1, -1)]

        for direction in directions_to_move:
            new_pos = (state[0] + direction[0], state[1] + direction[1])
            if not self.grid.is_occupied(new_pos[0], new_pos[1]):
                neighbors.append(new_pos)

        return neighbors

    def get_cost(self, node, child_node):
        """Gets the transition cost between two states. While it is known the cost to move left/right/up/down is 1,
        this accounts for moving diagonally, which has a cost of root 2"""
        return math.sqrt((node[0] - child_node[0]) ** 2 + (node[1] - child_node[1]) ** 2)


class Planner:
    """This class gets the occupancy grid info and calculates the shortest path using A* and its helper class.
    Publishes pose information and visualization markers along path."""
    def __init__(self):
        """Initialization function."""
        # setting up the publishers.
        self.marker_pub = rospy.Publisher("/markers", Marker, queue_size=10)
        self.pose_stamp_pub = rospy.Publisher("/pose_sequence", PoseStamped, queue_size=10)

        # only need to get grid data at initialization, dont need to subscribe
        grid_msg = rospy.wait_for_message("/map", OccupancyGrid, 3)
        self.grid = Grid(grid_msg.data, grid_msg.info.width, grid_msg.info.height, grid_msg.info.resolution)

        # sleep important for allowing time for registration to the ROS master.
        rospy.sleep(2)

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

"""""""RVIZ Functions"""""""""
    def publish_markers(self, path):
        """This method adds a marker at each point in the final path"""
        for mark in range(0, len(path)):
            marker_msg = Marker()
            marker_msg.header.stamp = rospy.Time.now()
            marker_msg.header.frame_id = "map"

            marker_msg.action = Marker.ADD
            marker_msg.type = Marker.SPHERE
            marker_msg.id = mark

            current_cell = path[mark]

            # convert cell index to position
            current_pose = self.grid.cell_to_point(current_cell[0], current_cell[1])

            # set orientation to 0
            marker_msg.pose.position.x = current_pose[0]
            marker_msg.pose.position.y = current_pose[1]

            quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
            marker_msg.pose.orientation.x = quaternion[0]
            marker_msg.pose.orientation.y = quaternion[1]
            marker_msg.pose.orientation.z = quaternion[2]
            marker_msg.pose.orientation.w = quaternion[3]

            # setting marker size
            marker_msg.color.a = 1
            marker_msg.scale.x = 0.05
            marker_msg.scale.y = 0.05
            marker_msg.scale.z = 0.05

            # set start node to blue, goal node to green, all other as red
            if mark == 0:
                marker_msg.color.b = 1.0
            elif mark == len(path) - 1:
                marker_msg.color.g = 1.0
            else:
                marker_msg.color.r = 1.0

            # publish markers
            self.marker_pub.publish(marker_msg)

            # add sleep to visualize path progression instead of points all at once
            time.sleep(.05)

    def publish_poses(self, path):
        """This method publishes the pose sequence. Adapted from ch.10 example code"""
        for pose in range(0, len(path)):
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"

            # convert cell index to position
            position = self.grid.cell_to_point(path[pose][0], path[pose][1])
            pose_msg.pose.position.x = position[0]
            pose_msg.pose.position.y = position[1]
            pose_msg.pose.position.z = 0

            if pose == 0:
                # set orientation at start to 0 radians
                quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
                pose_msg.pose.orientation.x = quaternion[0]
                pose_msg.pose.orientation.y = quaternion[1]
                pose_msg.pose.orientation.z = quaternion[2]
                pose_msg.pose.orientation.w = quaternion[3]
            else:
                # calculate new orientation based off angle between current and previous position
                prev_position = self.grid.cell_to_point(path[pose - 1][0], path[pose - 1][1])
                quaternion = self.calculate_orientation(prev_position, position)
                pose_msg.pose.orientation.x = quaternion[0]
                pose_msg.pose.orientation.y = quaternion[1]
                pose_msg.pose.orientation.z = quaternion[2]
                pose_msg.pose.orientation.w = quaternion[3]

            self.pose_stamp_pub.publish(pose_msg)
            time.sleep(.05)

    def delete_markers(self):
        """This is a helper function to delete old markers every time the program is rerun"""
        marker_msg = Marker()
        marker_msg.action = Marker.DELETEALL
        self.marker_pub.publish(marker_msg)

    def calculate_orientation(self, pos_i, pos_f):
        """Calculates orientation as quaternion using angle between initial and final point"""
        return tf.transformations.quaternion_from_euler(0, 0, math.atan2(pos_f[1] - pos_i[1], pos_f[0] - pos_i[0]))

"""""""Main"""""""""
def main():
    """main function."""
    rospy.init_node("planner")
    plan = Planner()

    path = plan.find_path((16.5, 16.5), (8, 25))  # change these values to find different paths

    print(path)

    plan.delete_markers()  # clear markers from before
    plan.publish_markers(path)
    plan.publish_poses(path)


if __name__ == "__main__":
    """ run the main function. """
    main()
