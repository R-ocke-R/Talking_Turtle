import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angular_speed = 0.5
        self.linear_speed = 0.5

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    move_turtle = MoveTurtle()
    try:
        rclpy.spin(move_turtle)
    except KeyboardInterrupt:
        pass
    move_turtle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()