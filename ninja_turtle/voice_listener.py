#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3
import speech_recognition as sr

class VoiceListener(Node):
    def __init__(self):
        super().__init__('voice_listener')
        self.pub = self.create_publisher(String, '/turtle_cmd', 10)
        self.get_logger().info("Voice listener ready. Speak commands like 'up', 'down', 'circle'...")

        # Initialize TTS
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)

        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        self.listen_loop()

    def speak(self, text):
        self.engine.say(text)
        self.engine.runAndWait()

    def listen_loop(self):
        self.get_logger().info("Listening...")
        while rclpy.ok():
            with self.microphone as source:
                audio = self.recognizer.listen(source)
            try:
                text = self.recognizer.recognize_google(audio).lower()
                self.get_logger().info(f"Heard: {text}")
                cmd = self.parse_command(text)
                if cmd:
                    msg = String()
                    msg.data = cmd
                    self.pub.publish(msg)
                    self.speak("Yes boss")
            except sr.UnknownValueError:
                pass  # Speech not understood
            except sr.RequestError as e:
                self.get_logger().error(f"Google error; {e}")
    
    def parse_command(self, text):
        # Map recognized text to commands
        for cmd in ["up", "down", "left", "right", "stop", "circle", "semi_circle", "full_circle"]:
            if cmd in text:
                return cmd
        return None


def main(args=None):
    rclpy.init(args=args)
    node = VoiceListener()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
