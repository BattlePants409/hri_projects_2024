#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class ObstacleAvoidanceTurnAway:
    def __init__(self):
        # Initialize the node
        rospy.init_node('obstacle_avoidance_turn_away', anonymous=True)

        # Publisher for movement commands
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber for laser scan data
        rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)

        # Set movement parameters
        self.move_cmd = Twist()
        self.forward_speed = 0.2  # Forward speed
        self.turn_speed = 1.0     # Turn speed for turn-away behavior
        self.distance_threshold = 0.5  # Distance threshold for obstacles

        self.turning_away = False  # Track if the robot is in turn-away mode
        self.rate = rospy.Rate(10)  # 10 Hz

    def laser_callback(self, data):
        # Skip further checks if already in turn-away mode
        if self.turning_away:
            return

        # Define front, left, and right ranges for obstacle detection
        front_range = min(data.ranges[len(data.ranges) // 2 - 20 : len(data.ranges) // 2 + 20])  # Wider front range
        left_range = min(data.ranges[:len(data.ranges) // 3])  # Left region
        right_range = min(data.ranges[-len(data.ranges) // 3:])  # Right region

        rospy.loginfo(f"Front: {front_range:.2f} m, Left: {left_range:.2f} m, Right: {right_range:.2f} m")

        # If an obstacle is detected in front or close to a side, activate turn-away behavior
        if front_range < self.distance_threshold or left_range < self.distance_threshold or right_range < self.distance_threshold:
            self.activate_turn_away(left_range, right_range)
        else:
            # Otherwise, move forward as normal
            self.move_cmd.linear.x = self.forward_speed
            self.move_cmd.angular.z = 0  # Stop turning
            rospy.loginfo("Moving forward")
            self.pub.publish(self.move_cmd)

    def activate_turn_away(self, left_range, right_range):
        """Activate the turn-away behavior to avoid obstacles."""
        # Stop forward motion
        self.move_cmd.linear.x = 0
        self.pub.publish(self.move_cmd)

        # Determine turn direction based on more space on either side
        if left_range > right_range:
            self.move_cmd.angular.z = -self.turn_speed  # Turn right
            rospy.loginfo("Turning right to avoid obstacle")
        else:
            self.move_cmd.angular.z = self.turn_speed   # Turn left
            rospy.loginfo("Turning left to avoid obstacle")

        # Publish the turn command for a short duration to commit to the turn
        self.turning_away = True  # Enter turn-away mode
        turn_duration = 0.2  # Fixed duration to turn away from obstacle (adjust as needed)
        turn_start_time = time.time()

        while time.time() - turn_start_time < turn_duration and not rospy.is_shutdown():
            self.pub.publish(self.move_cmd)
            self.rate.sleep()

        # Stop turning and exit turn-away mode
        self.move_cmd.angular.z = 0
        self.pub.publish(self.move_cmd)
        self.turning_away = False  # Exit turn-away mode

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        robot = ObstacleAvoidanceTurnAway()
        robot.run()
    except rospy.ROSInterruptException:
        pass

