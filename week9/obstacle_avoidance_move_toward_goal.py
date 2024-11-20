#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import math
import time

class MoveTowardPerson:
    def __init__(self):
        # Initialize the node
        rospy.init_node('move_toward_person', anonymous=True)

        # Publisher for movement commands
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber for laser scan data
        rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)

        # Subscriber for detected person position
        rospy.Subscriber('/person_position', PoseStamped, self.person_callback)

        # Movement parameters
        self.move_cmd = Twist()
        self.forward_speed = 0.2  # Forward speed
        self.turn_speed = 1.0     # Turning speed
        self.distance_threshold = 0.5  # Obstacle avoidance threshold
        self.goal_tolerance = 0.2  # Distance at which robot stops near the person

        # Person's position (updated dynamically)
        self.person_x = None
        self.person_y = None

        # Track if turning to avoid obstacle
        self.turning_away = False
        self.rate = rospy.Rate(10)  # 10 Hz

    def person_callback(self, msg):
        """Update the detected person's position."""
        self.person_x = msg.pose.position.x
        self.person_y = msg.pose.position.y

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

        # Goal-seeking behavior (move toward the person)
        if self.person_x is not None and self.person_y is not None:
            self.move_toward_person()
        else:
            # No person detected, stop moving
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = 0
            rospy.loginfo("No person detected")
            self.pub.publish(self.move_cmd)

    def move_toward_person(self):
        """Move the robot toward the detected person."""
        # Compute the direction to the person
        angle_to_person = math.atan2(self.person_y, self.person_x)
        distance_to_person = math.sqrt(self.person_x**2 + self.person_y**2)

        if distance_to_person < self.goal_tolerance:
            # Stop if close to the person
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = 0
            rospy.loginfo("Reached the person")
        else:
            # Move toward the person
            self.move_cmd.linear.x = self.forward_speed
            self.move_cmd.angular.z = angle_to_person  # Turn toward the person
            rospy.loginfo(f"Moving toward person: Distance {distance_to_person:.2f} m, Angle {angle_to_person:.2f} rad")

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
        robot = MoveTowardPerson()
        robot.run()
    except rospy.ROSInterruptException:
        pass

