import time

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
