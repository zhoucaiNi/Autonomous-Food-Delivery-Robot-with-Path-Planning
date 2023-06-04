from pathfinder import PathFinder 
from grid import Grid
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from heapq import heappush, heappop
import tf
from node import Node
import time
import math

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
        
        # print(start_cell, goal_cell)
        path_finder = PathFinder(self.grid, goal_cell)
        
        start_cell = path_finder.find_closest_valid_cell(start_cell)
        goal_cell = path_finder.find_closest_valid_cell(goal_cell)
        path_finder.goal_state = goal_cell

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
        else:
            
            print("No Valid Path")

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
            marker_msg.scale.x = 0.25
            marker_msg.scale.y = 0.25
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
