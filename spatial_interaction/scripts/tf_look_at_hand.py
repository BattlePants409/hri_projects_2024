#!/usr/bin/python3
import rospy
import math
import tf2_ros
from sensor_msgs.msg import JointState

def update_head_position(trans, msg, pub):
    """
    Updates the head position (HeadYaw and HeadPitch) based on the transform data.
    
    :param trans: The transform data containing the position of the hand.
    :param msg: The incoming joint state message.
    :param pub: The ROS publisher for joint states.
    """
    # Calculate yaw and pitch angles to point the head towards the hand
    x = trans.transform.translation.x
    y = trans.transform.translation.y
    z = trans.transform.translation.z
    
    # Head yaw (rotation around the vertical axis, left/right)
    head_yaw = math.atan2(y, x)

    # Head pitch (rotation up/down, relative to horizontal plane)
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
        pub.publish(new_msg)
        rospy.loginfo(f"HeadYaw: {head_yaw:.2f}, HeadPitch: {head_pitch:.2f}")

    except ValueError:
        rospy.logerr("Head joints not found in joint names list.")

def callback(msg):
    try:
        # Lookup transform from the head to the left hand
        trans = tfBuffer.lookup_transform('Head', 'l_gripper', rospy.Time())
        update_head_position(trans, msg, pub)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Transform lookup failed. Retrying...")

if __name__ == '__main__':
    rospy.init_node('tf2_look_at_hand')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Publisher for joint states
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.Subscriber('joint_state_input', JointState, callback)

    rospy.spin()

