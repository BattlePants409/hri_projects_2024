#!/usr/bin/python3
import rospy
import math
import tf2_ros
from sensor_msgs.msg import JointState

class AdjustHeadNode:
    def __init__(self):
        rospy.init_node('adjust_head_node')
        
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
        
        self.rate = rospy.Rate(10.0)
        
    def joint_state_callback(self, msg):
        # Try to look up the transform from the head to the left hand
        try:
            trans = self.tfBuffer.lookup_transform('Head', 'l_gripper', rospy.Time())
            self.update_head_position(msg, trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform lookup failed. Using default head position.")
            self.pub.publish(msg)

    def update_head_position(self, msg, trans):
        """
        Updates the head position (HeadYaw and HeadPitch) based on the transform data.
        """
        # Calculate yaw and pitch angles to point the head towards the hand
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z

        head_yaw = math.atan2(y, x)
        distance = math.sqrt(x**2 + y**2)
        head_pitch = -math.atan2(z, distance)

        # Create a new JointState message based on the incoming message
        new_msg = JointState()
        new_msg.header = msg.header
        new_msg.name = msg.name
        new_msg.position = list(msg.position)

        # Update only the head joints
        try:
            head_yaw_index = new_msg.name.index("HeadYaw")
            head_pitch_index = new_msg.name.index("HeadPitch")

            new_msg.position[head_yaw_index] = head_yaw
            new_msg.position[head_pitch_index] = head_pitch

            # Publish the modified joint states
            self.pub.publish(new_msg)
            rospy.loginfo(f"HeadYaw: {head_yaw:.2f}, HeadPitch: {head_pitch:.2f}")

        except ValueError:
            rospy.logerr("Head joints not found in joint names list.")

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = AdjustHeadNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

