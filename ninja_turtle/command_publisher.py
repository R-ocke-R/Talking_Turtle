#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(String, '/turtle_cmd', 10)
        self.get_logger().info("Type commands: up, down, left, right, stop, circle")

    def run(self):
        while True:
            cmd = input("Command: ")
            msg = String()
            msg.data = cmd
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
