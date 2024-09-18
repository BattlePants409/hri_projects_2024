#!/usr/bin/python3
# license removed for brevity
import rospy
import math
from sensor_msgs.msg import JointState
from time import sleep

def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    # Define the joint names
    joint_names = [
        "HeadYaw", "HeadPitch", "LHipYawPitch", "LHipRoll", "LHipPitch",
        "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipRoll", "RHipPitch",
        "RKneePitch", "RAnklePitch", "RAnkleRoll", "LShoulderPitch", "LShoulderRoll",
        "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand", "RShoulderPitch",
        "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"
    ]

    # Animation sequences

    # Nod Yes
    nod_yes_sequence = [
        [0 if name != "HeadPitch" else math.radians(20) for name in joint_names],  # HeadPitch up
        [0 if name != "HeadPitch" else math.radians(-20) for name in joint_names],  # HeadPitch down
        [0] * len(joint_names)  # Return to neutral
    ]

    # Bow
    bow_sequence = [
        [0 if name not in ["HeadPitch", "LHipPitch", "RHipPitch"] else math.radians(20) for name in joint_names],  # Head and torso forward
        [0] * len(joint_names)  # Return to neutral
    ]

    # Raise Both Arms
    raise_both_arms_sequence = [
        [0 if name not in ["LShoulderPitch", "RShoulderPitch"] else math.radians(-90) for name in joint_names],  # Arms up
        [0] * len(joint_names)  # Return to neutral
    ]

    # Combine all animations into a single sequence
    joint_positions_sequence = nod_yes_sequence + bow_sequence + raise_both_arms_sequence

    # Delay between each position change (in seconds)
    delay_between_changes = 2

    # Do each step in the combined sequence
    for joint_positions in joint_positions_sequence:
        # Create a JointState message
        js = JointState()
        js.header.stamp = rospy.get_rostime()
        js.header.frame_id = "Torso"
        js.name = joint_names
        js.position = joint_positions

        # Publish the joint states
        pub.publish(js)
        
        # Log the published joint state
        rospy.loginfo(js)

        # Delay before moving to the next joint state
        sleep(delay_between_changes)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

