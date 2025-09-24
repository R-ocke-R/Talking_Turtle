#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import time
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            String,
            '/turtle_cmd',
            self.command_callback,
            10)
        
        self.timer = None
        self.movement_start_time = None
        self.current_movement = None

        self.get_logger().info("Turtle Controller Ready! Listening for commands...")

    def command_callback(self, msg):
        cmd = msg.data.lower()
        twist = Twist()

        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

        if cmd == "up":
            twist.linear.x = 2.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.timer = self.create_timer(2.0, self.stop_movement)

        elif cmd == "down":
            twist.linear.x = -2.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.timer = self.create_timer(2.0, self.stop_movement)

        elif cmd == "left":
            twist.linear.x = 0.0
            twist.angular.z = 1.57
            self.publisher_.publish(twist)
            self.timer = self.create_timer(1.0, self.stop_movement)

        elif cmd == "right":
            twist.linear.x = 0.0
            twist.angular.z = -1.57
            self.publisher_.publish(twist)
            self.timer = self.create_timer(1.0, self.stop_movement)

        elif cmd == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

        elif cmd == "circle":
            # Continuous circular motion
            twist.linear.x = 10.0
            twist.angular.z = 7.0
            self.publisher_.publish(twist)

        elif cmd == "arc":
            # Semi-circle using timer (non-blocking)
            twist.linear.x = 2.0
            twist.angular.z = 1.5
            self.publisher_.publish(twist) 

        elif cmd == "clear":
            # Call the clear service to reset the turtlesim screen (non-blocking)
            
            client = self.create_client(Empty, '/clear')
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('Clear service not available')
                return
            req = Empty.Request()
            client.call_async(req)
            self.get_logger().info('Screen cleared!')

        elif cmd == "reset":
            # First reset turtle position to center
            client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('Teleport service not available')
                return
            req = TeleportAbsolute.Request()
            req.x = 5.544445
            req.y = 5.544445
            req.theta = 0.0
            client.call_async(req)
            # Then clear screen
            clear_client = self.create_client(Empty, '/clear')
            if not clear_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('Clear service not available')
                return
            clear_req = Empty.Request()
            clear_client.call_async(clear_req)
            self.get_logger().info('Turtle position reset and screen cleared!')

        else:
            self.get_logger().info(f"Unknown command: {cmd}")
            return

        self.get_logger().info(f"Executing command: {cmd}")

    def stop_movement(self):
        """Timer callback to stop movement"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        
        self.get_logger().info("Timed movement completed")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
