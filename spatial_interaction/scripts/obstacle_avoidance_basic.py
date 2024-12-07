#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceBasic:
    def __init__(self):
        # Initialize the node
        rospy.init_node('obstacle_avoidance_basic', anonymous=True)

        # Publisher for movement commands
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber for laser scan data
        rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)

        # Set movement parameters
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0.2  # Move forward at a constant speed
        self.distance_threshold = 0.5  # Distance threshold in meters

        self.rate = rospy.Rate(10)  # 10 Hz

    def laser_callback(self, data):
        # Check if any object is within the distance threshold
        min_distance = min(data.ranges)
        rospy.loginfo(f"Minimum distance to an obstacle: {min_distance:.2f} meters")

        if min_distance < self.distance_threshold:
            # Obstacle detected within threshold, stop the robot
            self.move_cmd.linear.x = 0
        else:
            # No obstacle within threshold, move forward
            self.move_cmd.linear.x = 0.2

        # Publish the movement command
        self.pub.publish(self.move_cmd)

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        robot = ObstacleAvoidanceBasic()
        robot.run()
    except rospy.ROSInterruptException:
        pass
