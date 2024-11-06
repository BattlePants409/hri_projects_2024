#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class FigureEightMovement:
    def __init__(self):
        rospy.init_node('figure_eight_movement', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.update_odometry)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.current_position = None
        self.current_orientation = 0.0
        self.previous_position = None
        self.cumulative_distance = 0.0

    def update_odometry(self, msg):
        # Update the robot's current position and orientation
        self.current_position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_orientation = euler_from_quaternion(orientation_list)

    def calculate_incremental_distance(self):
        """Calculate distance incrementally from the previous position to current position."""
        if self.previous_position is None:
            self.previous_position = self.current_position
            return 0.0
        dx = self.current_position.x - self.previous_position.x
        dy = self.current_position.y - self.previous_position.y
        incremental_distance = math.sqrt(dx**2 + dy**2)
        self.previous_position = self.current_position  # Update for next increment
        return incremental_distance

    def move_in_full_circle(self, linear_speed, angular_speed, circle_distance):
        """
        Move in a full circle based on cumulative distance.
        """
        # Ensure we have a valid current position before starting
        while self.current_position is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for odometry data...")
            rospy.sleep(0.1)

        # Reset cumulative distance tracking
        self.cumulative_distance = 0.0
        self.previous_position = self.current_position  # Set initial position
        move_cmd = Twist()
        move_cmd.linear.x = linear_speed
        move_cmd.angular.z = angular_speed

        while not rospy.is_shutdown():
            # Increment cumulative distance traveled
            self.cumulative_distance += self.calculate_incremental_distance()
            rospy.loginfo(f"Cumulative distance traveled: {self.cumulative_distance:.2f} meters (Target: {circle_distance:.2f} meters)")

            if self.cumulative_distance >= circle_distance:
                rospy.loginfo("Reached target cumulative distance for full circle. Stopping.")
                break  # Stop when the full circle distance is reached

            self.pub.publish(move_cmd)
            self.rate.sleep()

        # Stop the robot after completing the full circle
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0
        self.pub.publish(move_cmd)

    def move_figure_eight(self):
        # Define speed and full-circle distance
        radius = 1.0  # meters
        full_circle_distance = 2 * math.pi * radius  # Full circumference
        linear_speed = 0.2  # meters per second
        angular_speed = linear_speed / radius  # to create a turn with the given radius

        # Wait for initial odometry data
        while self.current_position is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for odometry data...")
            rospy.sleep(0.1)

        # First loop of the figure-eight
        rospy.loginfo("Starting first loop of figure-eight")
        self.move_in_full_circle(linear_speed, angular_speed, full_circle_distance)

        # Second loop of the figure-eight in the opposite direction
        rospy.loginfo("Starting second loop of figure-eight")
        self.move_in_full_circle(linear_speed, -angular_speed, full_circle_distance)

        rospy.loginfo("Figure-eight movement complete. Stopping the robot.")

if __name__ == '__main__':
    try:
        robot = FigureEightMovement()
        rospy.sleep(1)  # Wait briefly for odometry to initialize
        robot.move_figure_eight()
    except rospy.ROSInterruptException:
        pass

