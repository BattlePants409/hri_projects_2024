import rospy
from std_msgs.msg import String

class SpeechRepeaterNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('speech_repeater_node', anonymous=True)

        # Subscribe to the speech recognition topic
        rospy.Subscriber('speech_recognition/result', String, self.speech_callback)

        # Publisher to the text-to-speech topic
        self.tts_publisher = rospy.Publisher('tts/phrase', String, queue_size=10)

        rospy.loginfo("Speech Repeater Node started. Listening for speech...")

    def speech_callback(self, msg):
        # Log the received speech
        rospy.loginfo(f"Heard: {msg.data}")

        # Publish the received speech back to the TTS topic
        self.tts_publisher.publish(msg.data)

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        node = SpeechRepeaterNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Speech Repeater Node terminated.")

