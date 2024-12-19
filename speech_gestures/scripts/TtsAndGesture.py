import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class TTSAndGestureNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('tts_and_gesture_node', anonymous=True)

        # Publishers
        self.tts_publisher = rospy.Publisher('tts/phrase', String, queue_size=10)
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

        # Subscriber
        rospy.Subscriber('speech_input', String, self.speech_callback)

        rospy.loginfo("TTS and Gesture Node initialized and running.")

    def perform_wave(self):
        rospy.loginfo("Performing wave gesture")
        self.update_arm_position("RShoulderPitch", 1.0)
        self.update_arm_position("RShoulderRoll", 0.5)

    def perform_nod(self):
        rospy.loginfo("Performing nod gesture")
        self.update_head_position(0.0, 0.5)  # Nod down
        rospy.sleep(0.5)
        self.update_head_position(0.0, -0.5) # Nod up

    def perform_shake_head(self):
        rospy.loginfo("Performing shake head gesture")
        self.update_head_position(0.5, 0.0)  # Look right
        rospy.sleep(0.5)
        self.update_head_position(-0.5, 0.0) # Look left

    def update_head_position(self, yaw, pitch):
        try:
            msg = JointState()
            msg.name = ["HeadYaw", "HeadPitch"]
            msg.position = [yaw, pitch]
            self.joint_pub.publish(msg)
        except ValueError:
            rospy.logerr("Failed to update head position.")

    def update_arm_position(self, joint_name, position):
        try:
            msg = JointState()
            msg.name = [joint_name]
            msg.position = [position]
            self.joint_pub.publish(msg)
        except ValueError:
            rospy.logerr(f"Joint {joint_name} not found.")

    def speech_callback(self, msg):
        text = msg.data
        rospy.loginfo(f"Received speech input: {text}")

        # Publish the text to TTS
        self.tts_publisher.publish(text)

        # Check for gestures based on specific words
        if "hello" in text.lower():
            self.perform_wave()
        elif "yes" in text.lower():
            self.perform_nod()
        elif "no" in text.lower():
            self.perform_shake_head()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = TTSAndGestureNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("TTS and Gesture Node terminated.")

