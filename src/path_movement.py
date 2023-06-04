#!/usr/bin/env python

# Import of python modules.
import math  # use of pi

# Import of relevant libraries.
import rospy  # module for ROS APIs
from geometry_msgs.msg import Twist  # message type for velocity command.
from sensor_msgs.msg import LaserScan  # message type for laser measurement.
from nav_msgs.msg import Odometry

import tf  # library for transformations.

import numpy as np

# Constants
FREQUENCY = 10  # Hz.
LINEAR_SPEED = 0.22  # m/s, max velocity of the Turtlebot3 Burger
ANGULAR_SPEED = math.pi / 4

DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_ODOM_TOPIC = 'odom'


class RobotMotion:
    def __init__(self, frequency, linear_speed, angular_speed):
        """Initialization function."""

        # Setting up publishers/subscribers:
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist,
                                        queue_size=1)  # Publisher to send velocity commands (i.e. Twist)
        self._odom_sub = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self._odom_callback,
                                          queue_size=1)  # Subscriber to track the robot's odom location

        # Setting up parameters
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.rate = rospy.Rate(frequency)

        self.transform = [0, 0, 0]  # X, Y, and orientation of robot in odom frame

        # Sleep important for allowing time for registration to the ROS master.
        rospy.sleep(2)

    def _odom_callback(self, msg):
        # Convert orientation from quaternions to euler
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w])

        self.transform = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]

    def translate_straight(self, distance):
        # Using linear speed and distance, find duration of movement
        duration = distance / self.linear_speed  # T = d/v

        start_time = rospy.get_rostime()  # Start of movement
        while rospy.get_rostime() - start_time < rospy.Duration(duration) and not rospy.is_shutdown():
            # Send movement data to publisher:
            msg = Twist()
            msg.linear.x = self.linear_speed
            self._cmd_pub.publish(msg)
            self.rate.sleep()

    def rotate(self, angle):
        # Find if angle is negative or positive to determine direction of rotation
        angular_velocity = self.angular_speed
        if angle < 0:
            angular_velocity *= -1

        # Using angular speed and distance, find duration of movement
        duration = abs(angle) / self.angular_speed  # Time should not be negative, hence the abs()

        start_time = rospy.get_rostime()  # Start of movement
        while rospy.get_rostime() - start_time < rospy.Duration(duration) and not rospy.is_shutdown():
            # Send movement data to publisher
            msg = Twist()

            msg.angular.z = angular_velocity
            self._cmd_pub.publish(msg)  # send the speed to the publisher
            self.rate.sleep()

    def get_rotation_to_target(self, target):
        # Get angle from current position to target position
        theta = np.arctan2((target[1] - self.transform[1]), (target[0] - self.transform[0]))
        # Subtract from current orientation
        return self.transform[2] - theta

    def get_distance_to_target(self, target):
        return math.hypot(target[0] - self.transform[0], target[1] - self.transform[1])

    def follow_input_path(self, input_path):
        # Adjust the input path based on the robot's current position
        adjusted_path = []

        current_position = [self.transform[0], self.transform[1]]  # Current position of the robot

        # Adjust each point in the input path based on the current position
        for point in input_path:
            adjusted_point = [point[0] + current_position[0], point[1] + current_position[1]]
            print(adjusted_point)
            adjusted_path.append(adjusted_point)

        # Call the follow_path function with the adjusted path
        self.follow_path(adjusted_path)

    def follow_path(self, path):
        # Follow the given path
        for point in range(len(path) - 1):
            angle_to_rotate = self.get_rotation_to_target(path[point + 1])
            distance_to_move = self.get_distance_to_target(path[point + 1])

            self.rotate(angle_to_rotate)
            self.translate_straight(distance_to_move)


def main():
    """main function."""
    # 1st. initialization of node.
    rospy.init_node("robot_motion")

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(2)

    # Initialization of the class for robot motion
    mover = RobotMotion(FREQUENCY, LINEAR_SPEED, ANGULAR_SPEED)

    input_path = [(0, 0), (0, 1), (0, 2), (1, 2)]
    mover.follow_input_path(input_path)

    # If interrupted, send a stop command before interrupting.
    # rospy.on_shutdown(random_walk.stop)

    # Robot random walks.
    try:
        pass
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()
