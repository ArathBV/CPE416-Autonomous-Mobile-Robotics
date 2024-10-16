import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class DrawSquare(Node):

    def __init__(self):
        super().__init__('draw_square')

        self.publisher_ = self.create_publisher(
                Twist,
                '/turtle1/cmd_vel',
                10)

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.turning = False

        self.forward_msg = Twist()
        self.forward_msg.linear.x = 1.0
        self.forward_msg.linear.y = 0.0
        self.forward_msg.linear.z = 0.0
        self.forward_msg.angular.x = 0.0
        self.forward_msg.angular.y = 0.0
        self.forward_msg.angular.z = 0.0

        self.turn_msg = Twist()
        self.turn_msg.angular.x = 0.0
        self.turn_msg.angular.y = 0.0
        self.turn_msg.angular.z = 1.57
        self.turn_msg.linear.x = 0.0
        self.turn_msg.linear.y = 0.0
        self.turn_msg.linear.z = 0.0
        return

    def timer_callback(self):
        if (self.turning):
            self.get_logger().info('Robot is Turning!')
            self.publisher_.publish(self.turn_msg)
        else:
            self.get_logger().info('Robot is not Turning!')
            self.publisher_.publish(self.forward_msg)

        self.turning = not self.turning
        return

def main(args=None):
    rclpy.init(args=args)

    draw_square = DrawSquare()
    rclpy.spin(draw_square)
    draw_square.destroy_node()
    rclpy.shutdown()
    return

if __name__ == '__main__':
    main()
