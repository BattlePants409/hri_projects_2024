#!/usr/bin/python3
# license removed for brevity
import rospy
import math
from sensor_msgs.msg import JointState

def execute_sequence(pub, sequence, joint_names, delay):
    """
    Publishes a sequence of joint positions for the Nao robot.
    
    :param pub: The ROS publisher for joint states.
    :param sequence: A list of joint positions sequences to execute.
    :param joint_names: The names of the joints to move.
    :param delay: Delay (in seconds) between each position change.
    """
    for joint_positions in sequence:
        # Create a JointState message
        js = JointState()
        js.header.stamp = rospy.get_rostime()
        js.header.frame_id = "Torso"
        js.name = joint_names
        js.position = joint_positions

        # Publish the joint states
        pub.publish(js)
        rospy.loginfo(js)

        # Delay before moving to the next joint state
        rospy.sleep(delay)

def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    # Define the joint names
    joint_names = [
        "HeadYaw", "HeadPitch", "LHipYawPitch", "LHipRoll", "LHipPitch",
        "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipRoll", "RHipPitch",
        "RKneePitch", "RAnklePitch", "RAnkleRoll", "LShoulderPitch", "LShoulderRoll",
        "LElbowYaw", "LElbowRoll", "LHand", "RShoulderPitch",
        "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RHand"
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
        [0 if name not in ["LShoulderPitch", "RShoulderPitch"] else math.radians(-1.57) for name in joint_names],  # Arms up (90 degrees in radians)
        [0] * len(joint_names)  # Return to neutral
    ]

    # Delay between each position change (in seconds)
    delay_between_changes = 2

    # Main loop
    while not rospy.is_shutdown():
        # Execute each sequence
        execute_sequence(pub, nod_yes_sequence, joint_names, delay_between_changes)
        execute_sequence(pub, bow_sequence, joint_names, delay_between_changes)
        execute_sequence(pub, raise_both_arms_sequence, joint_names, delay_between_changes)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

