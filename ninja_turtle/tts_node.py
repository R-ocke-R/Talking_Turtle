import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3
import threading
import queue

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.subscription = self.create_subscription(
            String,
            '/speak_this',
            self.speak_callback,
            10)
        self.speech_queue = queue.Queue()
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)
        self.engine.setProperty('volume', 0.9)
        self.get_logger().info('TTS Node initialized and ready to speak!')

        # Start a thread to process speech requests
        self.speech_thread = threading.Thread(target=self.process_speech)
        self.speech_thread.daemon = True
        self.speech_thread.start()

    def speak_callback(self, msg):
        self.speech_queue.put(msg.data)

    def process_speech(self):
        while True:
            text = self.speech_queue.get()
            if text:
                self.get_logger().info(f'Speaking: {text}')
                self.engine.say(text)
                self.engine.runAndWait()

def main(args=None):
    rclpy.init(args=args)
    tts_node = TTSNode()
    rclpy.spin(tts_node)
    tts_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()