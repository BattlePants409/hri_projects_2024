#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
import math
import time

class JoinGroupAvoidingCenter:
    def __init__(self):
        # Initialize the node
        rospy.init_node('join_group_avoiding_center', anonymous=True)

        # Publisher for movement commands
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber for laser scan data
        rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)

        # Subscriber for detected group positions
        rospy.Subscriber('/group_positions', PoseArray, self.group_callback)

        # Movement parameters
        self.move_cmd = Twist()
        self.forward_speed = 0.2  # Forward speed
        self.turn_speed = 1.0     # Turning speed
        self.distance_threshold = 0.5  # Obstacle avoidance threshold
        self.goal_tolerance = 0.5  # Distance at which robot stops near the goal

        # Group information
        self.group_x = None
        self.group_y = None
        self.group_radius = 0.0
        self.join_angle = 0.0  # Angle to the joining point on the circle's edge

        # Track if turning to avoid obstacle
        self.turning_away = False
        self.rate = rospy.Rate(10)  # 10 Hz

    def group_callback(self, msg):
        """Update the centroid and radius of the detected group."""
        if len(msg.poses) == 0:
            self.group_x = None
            self.group_y = None
            self.group_radius = 0.0
            return

        # Calculate the centroid of the group
        sum_x = 0.0
        sum_y = 0.0
        max_distance = 0.0  # Track the farthest member for radius estimation
        for pose in msg.poses:
            sum_x += pose.position.x
            sum_y += pose.position.y

        self.group_x = sum_x / len(msg.poses)
        self.group_y = sum_y / len(msg.poses)

        # Calculate the radius as the distance to the farthest group member
        for pose in msg.poses:
            distance = math.sqrt((pose.position.x - self.group_x)**2 + (pose.position.y - self.group_y)**2)
            max_distance = max(max_distance, distance)

        self.group_radius = max_distance + 0.5  # Add margin for safe joining

    def laser_callback(self, data):
        """Handle laser scan data for obstacle avoidance and navigation."""
        if self.turning_away:
            return

        # Obstacle detection
        front_range = min(data.ranges[len(data.ranges) // 2 - 20 : len(data.ranges) // 2 + 20])
        left_range = min(data.ranges[:len(data.ranges) // 3])
        right_range = min(data.ranges[-len(data.ranges) // 3:])

        # If obstacles are too close, activate turn-away behavior
        if front_range < self.distance_threshold or left_range < self.distance_threshold or right_range < self.distance_threshold:
            self.activate_turn_away(left_range, right_range)
            return

        # Goal-seeking behavior (move toward the joining point)
        if self.group_x is not None and self.group_y is not None:
            self.move_to_joining_point()
        else:
            # No group detected, stop moving
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = 0
            rospy.loginfo("No group detected")
            self.pub.publish(self.move_cmd)

    def move_to_joining_point(self):
        """Move the robot toward an empty space on the group's circle."""
        # Calculate the joining point on the group's radius
        join_x = self.group_x + self.group_radius * math.cos(self.join_angle)
        join_y = self.group_y + self.group_radius * math.sin(self.join_angle)

        # Compute the direction to the joining point
        angle_to_join = math.atan2(join_y, join_x)
        distance_to_join = math.sqrt(join_x**2 + join_y**2)

        if distance_to_join < self.goal_tolerance:
            # Stop if close to the joining point
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = 0
            rospy.loginfo("Joined the group at the specified location")
        else:
            # Move toward the joining point
            self.move_cmd.linear.x = self.forward_speed
            self.move_cmd.angular.z = angle_to_join  # Turn toward the joining point
            rospy.loginfo(f"Moving to join group: Distance {distance_to_join:.2f} m, Angle {angle_to_join:.2f} rad")

        # Publish the movement command
        self.pub.publish(self.move_cmd)

    def activate_turn_away(self, left_range, right_range):
        """Activate turn-away behavior to avoid obstacles."""
        self.move_cmd.linear.x = 0  # Stop forward motion
        self.pub.publish(self.move_cmd)

        # Determine turn direction based on more space
        if left_range > right_range:
            self.move_cmd.angular.z = -self.turn_speed  # Turn right
            rospy.loginfo("Turning right to avoid obstacle")
        else:
            self.move_cmd.angular.z = self.turn_speed  # Turn left
            rospy.loginfo("Turning left to avoid obstacle")

        # Publish turn command for a short duration
        self.turning_away = True
        turn_duration = 0.2
        turn_start_time = time.time()

        while time.time() - turn_start_time < turn_duration and not rospy.is_shutdown():
            self.pub.publish(self.move_cmd)
            self.rate.sleep()

        # Stop turning
        self.move_cmd.angular.z = 0
        self.pub.publish(self.move_cmd)
        self.turning_away = False

    def run(self):
        """Keep the node running."""
        rospy.spin()

if __name__ == '__main__':
    try:
        robot = JoinGroupAvoidingCenter()
        robot.run()
    except rospy.ROSInterruptException:
        pass

