#!/usr/bin/env python

#twist mux is used to help with ros topics for requires downloading to root on every launch 
# download guide below 
# sudo apt update
# sudo apt install ros-humble-twist-mux
# config and launch folders were copied into this workspace from twistmux package
#https://index.ros.org/r/twist_mux/

# This code was adapted from the github page 
#https://github.com/LCAS/teaching/tree/lcas_humble/cmp3103m_ros2_code_fragments

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class roam(Node):
    """
    A simple Roaming ROS2 node. Subscribes to "/scan" and sends velocity commands to "roaming_vel".
    """

    min_distance = 0.1 # stay at least 30cm away from obstacles
    turn_speed = 0.3    # rad/s, turning speed in case of obstacle
    forward_speed = 0.25 # m/s, speed with which to go forward if the space is clear
    scan_segment = 60   # degrees, the size of the left and right laser segment to search for obstacles

    def __init__(self):
        """
        Initialiser, setting up subscription to "/scan" and creates publisher for "/cmd_vel"
        """
        super().__init__('roam')
        self.laser_sub = self.create_subscription(LaserScan,"/scan",self.callback, 1)
        self.twist_pub = self.create_publisher(Twist,'roaming_vel', 1)

    def min_range(self, range):
        # initialise a variable to hold the smallest value found so far as positive infinity.
        min_range = math.inf

        # iterate over each element of the array.
        for v in range:

            # check if the current value is less than the smallest value found so far.
            if v < min_range:

                # if it is, update the smallest value found so far to be the current value.
                min_range = v

        # return the smallest value found in the array.
        return min_range

    def callback(self, data):
        """
        This function is a callback that gets called for every LaserScan received. 
        It checks if there are any obstacles within the specified segment of the LaserScan data. 
        If an obstacle is detected, it turns the robot accordingly. If the path is clear, it moves forward.
        """

        # First, identify the nearest obstacle in the right and left 45-degree segments of the laser scanner.
        # To achieve this, the function calls the 'min_range' function defined elsewhere, which returns the minimum value
        # of an array of ranges. The 'data.ranges' array contains the ranges (distances) of obstacles detected by the 
        # laser scanner. The [:self.scan_segment] and [-self.scan_segment:] syntax is used to slice the ranges array 
        # and only look at the ranges within the specified segments.
        min_range_right = self.min_range(data.ranges[:self.scan_segment])
        min_range_left = self.min_range(data.ranges[-self.scan_segment:])

        # Next, initialise a Twist object to hold the linear and angular velocities of the robot.
        twist = Twist()

        # If an obstacle is detected within the right segment of the laser scanner, turn left.
        if min_range_right < self.min_distance:
            self.get_logger().info('turning left')
            twist.angular.z = -self.turn_speed

        # If an obstacle is detected within the left segment of the laser scanner, turn right.
        elif min_range_left < self.min_distance:
            self.get_logger().info('turning right')
            twist.angular.z = self.turn_speed

        # MAIN USE OF THIS SCRIPT 
        # If no obstacles are detected within the specified segments, move forward.
        else:
            self.get_logger().info('going straight')
            twist.linear.x = self.forward_speed

        # Finally, publish the Twist message to the robot's topic so that it can execute the desired motion.
        self.twist_pub.publish(twist)  

def main(args=None):
    print('Starting roaming.py.')

    try:
        # Initialise the ROS Python subsystem
        rclpy.init()
        # create the Node object we want to run
        node = roam()
        # keep the node running until it is stopped (e.g. by pressing Ctrl-C)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('interrupted')
    finally:
        # we use "finally" here, to ensure that everything is correctly tidied up,
        # in case of any exceptions happening.
        node.destroy_node()

if __name__ == '__main__':
    main()