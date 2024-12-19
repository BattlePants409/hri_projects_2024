import rospy
import math
import tf2_ros
from sensor_msgs.msg import JointState

class SpeechCommandGestureNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('speech_command_gesture_node', anonymous=True)

        # TF2 Buffer and Listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Publisher for joint states
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)

        rospy.loginfo("Speech Command Gesture Node started. Listening for commands...")

    def perform_wave(self, msg):
        rospy.loginfo("Performing wave gesture")
        # Simulate a wave by modifying arm joint states
        self.update_arm_position(msg, "RShoulderPitch", 1.0)  # Example values
        self.update_arm_position(msg, "RShoulderRoll", 0.5)

    def perform_nod(self, msg):
        rospy.loginfo("Performing nod gesture")
        # Simulate a nod by modifying head joint states
        self.update_head_position(msg, 0.0, 0.5)  # Nod down
        rospy.sleep(0.5)
        self.update_head_position(msg, 0.0, -0.5) # Nod up

    def perform_shake_head(self, msg):
        rospy.loginfo("Performing shake head gesture")
        # Simulate a head shake by modifying head joint states
        self.update_head_position(msg, 0.5, 0.0)  # Look right
        rospy.sleep(0.5)
        self.update_head_position(msg, -0.5, 0.0) # Look left

    def update_head_position(self, msg, yaw, pitch):
        try:
            new_msg = JointState()
            new_msg.header = msg.header
            new_msg.name = msg.name
            new_msg.position = list(msg.position)

            head_yaw_index = new_msg.name.index("HeadYaw")
            head_pitch_index = new_msg.name.index("HeadPitch")

            new_msg.position[head_yaw_index] = yaw
            new_msg.position[head_pitch_index] = pitch

            self.pub.publish(new_msg)
        except ValueError:
            rospy.logerr("Head joints not found in joint names list.")

    def update_arm_position(self, msg, joint_name, position):
        try:
            new_msg = JointState()
            new_msg.header = msg.header
            new_msg.name = msg.name
            new_msg.position = list(msg.position)

            joint_index = new_msg.name.index(joint_name)
            new_msg.position[joint_index] = position

            self.pub.publish(new_msg)
        except ValueError:
            rospy.logerr(f"Joint {joint_name} not found in joint names list.")

    def speech_callback(self, msg):
        speech = msg.data.lower().strip()

        if speech in ['hi', 'hello']:
            self.perform_wave(msg)
        elif speech == 'yes':
            self.perform_nod(msg)
        elif speech == 'no':
            self.perform_shake_head(msg)
        else:
            rospy.loginfo(f"Unrecognized command: '{speech}'")

    def run(self):
        rospy.Subscriber('speech_recognition/result', String, self.speech_callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        node = SpeechCommandGestureNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Speech Command Gesture Node terminated.")

