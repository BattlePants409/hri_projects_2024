#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class SquareMovement:
    def __init__(self):
        # Initialize node, publisher, and subscriber
        rospy.init_node('square_movement', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.update_odometry)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.current_position = None
        self.current_orientation = 0.0
        self.previous_position = None
        self.cumulative_distance = 0.0

    def update_odometry(self, msg):
        # Update current position and orientation from odometry
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

    def move_distance(self, distance):
        """Move forward by a specified distance in meters."""
        # Ensure valid current position before starting
        while self.current_position is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for odometry data...")
            rospy.sleep(0.1)

        # Reset cumulative distance tracking
        self.cumulative_distance = 0.0
        self.previous_position = self.current_position  # Set initial position
        move_cmd = Twist()
        move_cmd.linear.x = 0.2  # Set a constant forward speed

        while not rospy.is_shutdown():
            # Increment cumulative distance traveled
            self.cumulative_distance += self.calculate_incremental_distance()
            rospy.loginfo(f"Cumulative distance traveled: {self.cumulative_distance:.2f} meters (Target: {distance:.2f} meters)")

            if self.cumulative_distance >= distance:
                rospy.loginfo("Reached target distance. Stopping.")
                break  # Stop once the required distance is reached

            self.pub.publish(move_cmd)
            self.rate.sleep()

        # Stop the robot after moving the required distance
        move_cmd.linear.x = 0
        self.pub.publish(move_cmd)

    def rotate_angle(self, angle):
        """Rotate by a specified angle in radians."""
        start_orientation = self.current_orientation
        target_orientation = start_orientation + angle

        # Normalize the target angle to stay within [-pi, pi]
        target_orientation = (target_orientation + math.pi) % (2 * math.pi) - math.pi

        move_cmd = Twist()
        move_cmd.angular.z = 0.5  # Set a constant turning speed

        while not rospy.is_shutdown():
            # Calculate shortest angle difference
            angle_diff = target_orientation - self.current_orientation
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Wrap within [-pi, pi]

            rospy.loginfo(f"Current angle difference: {angle_diff:.2f} radians (Target: {angle:.2f} radians)")

            if abs(angle_diff) < 0.05:  # Close enough to target angle
                rospy.loginfo("Reached target angle. Stopping.")
                break

            self.pub.publish(move_cmd)
            self.rate.sleep()

        # Stop the robot after rotating the required angle
        move_cmd.angular.z = 0
        self.pub.publish(move_cmd)

    def move_in_square(self, side_length):
        """Move the robot in a square pattern with specified side length."""
        for _ in range(4):  # Repeat four times for a square
            rospy.loginfo("Moving forward along a side of the square")
            self.move_distance(side_length)  # Move forward

            rospy.loginfo("Turning 90 degrees")
            self.rotate_angle(math.pi / 2)  # Turn 90 degrees

        rospy.loginfo("Square movement complete. Stopping the robot.")

if __name__ == '__main__':
    try:
        robot = SquareMovement()
        rospy.sleep(1)  # Give time for odometry to initialize
        robot.move_in_square(1.0)  # Move in a square with 1 meter sides
    except rospy.ROSInterruptException:
        pass

