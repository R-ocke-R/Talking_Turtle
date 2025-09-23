#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import time

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            String,
            '/turtle_cmd',
            self.command_callback,
            10)
        self.get_logger().info("Turtle Controller Ready! Listening for commands...")

    def command_callback(self, msg):
        cmd = msg.data.lower()
        twist = Twist()

        if cmd == "up":
            twist.linear.x = 2.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

        elif cmd == "down":
            twist.linear.x = -2.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

        elif cmd == "left":
            twist.linear.x = 0.0
            twist.angular.z = 2.0
            self.publisher_.publish(twist)

        elif cmd == "right":
            twist.linear.x = 0.0
            twist.angular.z = -2.0
            self.publisher_.publish(twist)

        elif cmd == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

        elif cmd == "circle":
            twist.linear.x = 2.0
            twist.angular.z = 1.5
            self.publisher_.publish(twist)

        elif cmd == "semi_circle":
            # semi-circle: rotate ~180 degrees
            twist.linear.x = 2.0
            twist.angular.z = 1.5
            self.publisher_.publish(twist)
            time.sleep(3.0)  # adjust this for correct arc length
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

        elif cmd == "full_circle":
            # full-circle: rotate 360 degrees
            twist.linear.x = 2.0
            twist.angular.z = 1.5
            self.publisher_.publish(twist)
            time.sleep(6.0)  # adjust this for full circle
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

        else:
            self.get_logger().info(f"Unknown command: {cmd}")
            return

        self.get_logger().info(f"Executing command: {cmd}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




