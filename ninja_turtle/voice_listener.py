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
        self.speech_pub = self.create_publisher(String, '/recognized_speech', 10)
        # Initialize TTS
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)
        greeting = (
            "Hi there, I'm the masked ninja turtle, here to follow your drawing steps. "
            # "Use your voice to make art on the screen sim! Currently, I can do the following: "
            # "move up, down, left, right, stop, draw an arc, and clear/reset the screen. "
            # "Just say the command and I'll do my best!"
        )
        self.speak(greeting)
        self.get_logger().info("Voice listener ready. Speak commands like 'up', 'down', 'circle'...")

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

                # Publish raw recognized speech for game manager
                speech_msg = String()
                speech_msg.data = text
                self.speech_pub.publish(speech_msg)

                cmd = self.parse_command(text)
                if cmd:
                    msg = String()
                    msg.data = cmd
                    self.pub.publish(msg)
                    # More human acknowledgment
                    ack_map = {
                        "up": "Moving up!",
                        "down": "Moving down!",
                        "left": "Turning left!",
                        "right": "Turning right!",
                        "stop": "Stopping now!",
                        "arc": "Drawing an arc!",
                        "circle": "Drawing a full circle!",
                        "clear": "Resetting the screen!",
                        "reset": "Resetting my position!"
                    }
                    ack = ack_map.get(cmd, f"Command '{cmd}' received!")
                    #self.speak(ack)
            except sr.UnknownValueError:
                pass  # Speech not understood
            except sr.RequestError as e:
                self.get_logger().error(f"Google error; {e}")
    
    def parse_command(self, text):
        # Map recognized text to commands
        command_map = {
            "up": ["up", "forward", "move forward", "go up"],
            "down": ["down", "backward", "move backward", "go down", "go back", "back"],
            "left": ["left", "turn left", "go left"],
            "right": ["right", "turn right", "go right"],
            "stop": ["stop", "halt", "freeze", "whoa"],
            "arc": ["arc", "draw arc", "make arc"],
            "circle": ["circle", "full circle", "draw full circle", "complete circle"],
            "clear": ["clear", "clear screen", "erase", "start over"],
            "reset": ["reset", "reset position", "center", "go to center", "teleport"]
        }
        
        for cmd_key, phrases in command_map.items():
            for phrase in phrases:
                if phrase in text:
                    return cmd_key
        return None


def main(args=None):
    rclpy.init(args=args)
    node = VoiceListener()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
