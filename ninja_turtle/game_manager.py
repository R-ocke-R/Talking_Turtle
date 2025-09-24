#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class GameManager(Node):
    def __init__(self):
        super().__init__('game_manager')
        self.state = 'idle'
        self.shape = None
        self.step = 0
        self.speech_sub = self.create_subscription(
            String,
            '/recognized_speech',
            self.speech_callback,
            10
        )
        self.turtle_cmd_pub = self.create_publisher(String, '/turtle_cmd', 10)
        self.speak_pub = self.create_publisher(String, '/speak_this', 10)

        # Fun facts about shapes
        self.square_facts = [
            "Did you know? A square is a special type of rectangle with all sides equal!",
            "Fun fact: All four angles in a square are exactly 90 degrees!",
            "Cool fact: You can fit exactly 4 quarter circles into a square!",
            "Amazing fact: A square has the most sides of any regular polygon that can tile a plane!",
            "Did you know? The ancient Egyptians used squares in their architecture!",
            "Fun fact: A square has rotational symmetry of 90 degrees!",
            "Cool fact: Every square is also a rhombus, but not every rhombus is a square!",
            "Amazing fact: The area of a square doubles when you double the side length!",
            "Did you know? Squares appear in nature, like in honeycombs and crystals!",
            "Fun fact: A square has the smallest perimeter for a given area among all rectangles!"
        ]

        self.rectangle_facts = [
            "Did you know? A rectangle is a four-sided shape with opposite sides equal!",
            "Fun fact: All four angles in a rectangle are exactly 90 degrees!",
            "Cool fact: The longest side of a rectangle is called the length, the shorter is width!",
            "Amazing fact: A square is a special type of rectangle!",
            "Did you know? Rectangles are everywhere - doors, books, phones are all rectangles!",
            "Fun fact: The area of a rectangle is length times width!",
            "Cool fact: You can divide any rectangle into two right triangles!",
            "Amazing fact: Rectangles have rotational symmetry of 180 degrees!",
            "Did you know? The ancient Greeks studied rectangles in geometry!",
            "Fun fact: A rectangle with equal diagonals is actually a square!"
        ]

    def speech_callback(self, msg):
        text = msg.data.strip().lower()
        if self.state == 'idle' and ("let's draw" in text or "draw with me" in text or "let's play" in text or "let's play a game" in text):
            self.start_game()
        elif self.state == 'choose_shape':
            if 'square' in text:
                self.shape = 'square'
                self.speak_pub.publish(String(data="Great choice! A square has four equal sides. To draw the first side, please say 'up'."))
                self.state = 'draw_square'
                self.step = 0
            elif 'rectangle' in text:
                self.shape = 'rectangle'
                self.speak_pub.publish(String(data="Great choice! A rectangle has two long sides and two short sides. To draw the first side, please say 'up'."))
                self.state = 'draw_rectangle'
                self.step = 0
        elif self.state == 'draw_square':
            self.handle_square(text)
        elif self.state == 'draw_rectangle':
            self.handle_rectangle(text)

    def start_game(self):
        self.turtle_cmd_pub.publish(String(data="reset"))
        self.turtle_cmd_pub.publish(String(data="clear"))
        
        self.speak_pub.publish(String(data="Awesome! I'd love to draw with you. What should we learn to draw today: a square or a rectangle?"))
        self.state = 'choose_shape'

    def reset_game(self):
        self.state = 'idle'
        self.shape = None
        self.step = 0

    def handle_square(self, text):
        expected_commands = ['up', 'right', 'up', 'right', 'up', 'right', 'up']
        prompts = [
            "Perfect! Now say 'right' to make the corner.",
            "Great! Now say 'up' to draw the next side.",
            "Excellent! Say 'right' to turn the corner again.",
            "You're doing great! Say 'up' for the next side.",
            "Almost there! Say 'right' for the final corner.",
            "Final step! Say 'up' to complete the square."
        ]

        if self.step < len(expected_commands) and expected_commands[self.step] in text:
            # Send the turtle command
            self.turtle_cmd_pub.publish(String(data=expected_commands[self.step]))

            # Speak next prompt or finish
            if self.step < len(prompts):
                self.speak_pub.publish(String(data=prompts[self.step]))
            else:
                completion_msg = f"We did it! We drew a perfect square together! {random.choice(self.square_facts)} What should we do next?"
                self.speak_pub.publish(String(data=completion_msg))
                self.reset_game()
                return

            self.step += 1

    def handle_rectangle(self, text):
        expected_commands = ['up', 'up', 'right', 'up', 'right', 'up', 'up', 'right', 'up']
        prompts = [
            "Great! Say 'up' again to complete this side.",
            "Perfect! Now say 'right' to make the corner.",
            "Good! Now say 'up' for the next side.",
            "Excellent! Say 'right' to turn again.",
            "You're doing great! Say 'up' for the next part.",
            "Almost there! Say 'up' again.",
            "Final corner! Say 'right'.",
            "Last step! Say 'up' to complete the rectangle."
        ]

        if self.step < len(expected_commands) and expected_commands[self.step] in text:
            # Send the turtle command
            self.turtle_cmd_pub.publish(String(data=expected_commands[self.step]))

            # Speak next prompt or finish
            if self.step < len(prompts):
                self.speak_pub.publish(String(data=prompts[self.step]))
            else:
                completion_msg = f"We did it! We drew a perfect rectangle together! {random.choice(self.rectangle_facts)} What should we do next?"
                self.speak_pub.publish(String(data=completion_msg))
                self.reset_game()
                return

            self.step += 1

def main(args=None):
    rclpy.init(args=args)
    node = GameManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
