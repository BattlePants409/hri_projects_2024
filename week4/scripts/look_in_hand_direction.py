#!/usr/bin/python3
import rospy
import math
import tf2_ros
from sensor_msgs.msg import JointState

class LookInHandDirectionNode:
    def __init__(self):
        rospy.init_node('look_in_hand_direction_node')
        
        # Define the joint names used in the Nao robot
        self.joint_names = [
            "HeadYaw", "HeadPitch", "LHipYawPitch", "LHipRoll", "LHipPitch",
            "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipRoll", "RHipPitch",
            "RKneePitch", "RAnklePitch", "RAnkleRoll", "LShoulderPitch", "LShoulderRoll",
            "LElbowYaw", "LElbowRoll", "LHand", "RShoulderPitch",
            "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RHand"
        ]
        
        # TF2 Buffer and Listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Publisher and Subscriber
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rospy.Subscriber('joint_state_input', JointState, self.joint_state_callback)

        # To store the last known head position
        self.last_head_yaw = 0.0
        self.last_head_pitch = 0.0
        
        self.rate = rospy.Rate(10.0)
        
    def joint_state_callback(self, msg):
        try:
            # Get the transform and orientation of the hand relative to the head
            trans = self.tfBuffer.lookup_transform('Head', 'l_gripper', rospy.Time())
            self.look_in_hand_direction(msg, trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform lookup failed. Retaining last known head position.")
            self.retain_last_head_position(msg)

    def look_in_hand_direction(self, msg, trans):
        """
        Adjusts the head position to look in the direction the hand is pointing.
        """
        # Get the hand's position and orientation
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z

        # Extract the hand's orientation as a quaternion
        q = trans.transform.rotation

        # Convert quaternion to a forward direction vector
        forward_x = 2 * (q.x * q.z + q.w * q.y)
        forward_y = 2 * (q.y * q.z - q.w * q.x)
        forward_z = 1 - 2 * (q.x * q.x + q.y * q.y)

        # Calculate the target point 1 meter in front of the hand
        point_x = x + forward_x
        point_y = y + forward_y
        point_z = z + forward_z

        # Calculate the yaw and pitch angles needed to look at the target point
        target_yaw = math.atan2(point_y, point_x)
        distance = math.sqrt(point_x**2 + point_y**2)
        target_pitch = -math.atan2(point_z, distance)

        # Store the calculated angles as the last known position
        self.last_head_yaw = target_yaw
        self.last_head_pitch = target_pitch

        # Create a new JointState message based on the incoming message
        new_msg = JointState()
        new_msg.header = msg.header
        new_msg.name = msg.name
        new_msg.position = list(msg.position)

        # Update only the head joints
        try:
            head_yaw_index = new_msg.name.index("HeadYaw")
            head_pitch_index = new_msg.name.index("HeadPitch")

            new_msg.position[head_yaw_index] = target_yaw
            new_msg.position[head_pitch_index] = target_pitch

            # Publish the modified joint states
            self.pub.publish(new_msg)
            rospy.loginfo(f"Target Yaw: {target_yaw:.2f}, Target Pitch: {target_pitch:.2f}")

        except ValueError:
            rospy.logerr("Head joints not found in joint names list.")

    def retain_last_head_position(self, msg):
        """
        Retains the last known head position when transform data is unavailable.
        """
        new_msg = JointState()
        new_msg.header = msg.header
        new_msg.name = msg.name
        new_msg.position = list(msg.position)

        try:
            head_yaw_index = new_msg.name.index("HeadYaw")
            head_pitch_index = new_msg.name.index("HeadPitch")

            # Set the head joints to the last known position
            new_msg.position[head_yaw_index] = self.last_head_yaw
            new_msg.position[head_pitch_index] = self.last_head_pitch

            # Publish the joint states with the retained head position
            self.pub.publish(new_msg)

        except ValueError:
            rospy.logerr("Head joints not found in joint names list.")

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = LookInHandDirectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

