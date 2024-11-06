#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class TriangleMovement:
    def __init__(self):
        rospy.init_node('triangle_movement', anonymous=True)
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

    def move_forward(self, linear_speed, side_length):
        """
        Move forward for a specified side length using cumulative distance.
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

        while not rospy.is_shutdown():
            # Increment cumulative distance traveled
            self.cumulative_distance += self.calculate_incremental_distance()
            rospy.loginfo(f"Cumulative distance traveled: {self.cumulative_distance:.2f} meters (Target: {side_length:.2f} meters)")

            if self.cumulative_distance >= side_length:
                rospy.loginfo("Reached target distance for side. Stopping.")
                break  # Stop when the side length is reached

            self.pub.publish(move_cmd)
            self.rate.sleep()

        # Stop the robot after completing the side
        move_cmd.linear.x = 0
        self.pub.publish(move_cmd)

    def turn(self, angular_speed, turn_angle):
        """
        Turn the robot by a specified angle (in radians).
        """
        start_orientation = self.current_orientation
        target_orientation = start_orientation + turn_angle

        # Normalize the target angle to stay within [-pi, pi]
        target_orientation = (target_orientation + math.pi) % (2 * math.pi) - math.pi

        move_cmd = Twist()
        move_cmd.angular.z = angular_speed

        while not rospy.is_shutdown():
            # Calculate shortest angle difference
            angle_diff = target_orientation - self.current_orientation
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Wrap within [-pi, pi]

            rospy.loginfo(f"Current angle difference: {angle_diff:.2f} radians (Target: {turn_angle:.2f} radians)")

            if abs(angle_diff) < 0.05:  # Close enough to target angle
                break

            self.pub.publish(move_cmd)
            self.rate.sleep()

        # Stop the robot after completing the turn
        move_cmd.angular.z = 0
        self.pub.publish(move_cmd)

    def move_triangle(self):
        # Define movement parameters for the triangle
        side_length = 2.0  # meters
        linear_speed = 0.2  # meters per second
        angular_speed = 0.5  # radians per second
        turn_angle = 2 * math.pi / 3  # 120 degrees in radians

        # Wait for initial odometry data
        while self.current_position is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for odometry data...")
            rospy.sleep(0.1)

        # Move in a triangle pattern
        for _ in range(3):
            rospy.loginfo("Moving forward along a side of the triangle")
            self.move_forward(linear_speed, side_length)

            rospy.loginfo("Turning 120 degrees")
            self.turn(angular_speed, turn_angle)

        rospy.loginfo("Triangle movement complete. Stopping the robot.")

if __name__ == '__main__':
    try:
        robot = TriangleMovement()
        rospy.sleep(1)  # Wait briefly for odometry to initialize
        robot.move_triangle()
    except rospy.ROSInterruptException:
        pass
