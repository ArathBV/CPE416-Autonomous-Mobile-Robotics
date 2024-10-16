import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class DrawSpiral(Node):

    def __init__(self):
        super().__init__('draw_spiral')

        self.publisher_ = self.create_publisher(
                Twist,
                'turtle1/cmd_vel',
                10)

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


        self.turning = False

        self.spin_msg = Twist()
        self.spin_msg.linear.x = 1.5
        self.spin_msg.angular.z = 0.05

    def timer_callback(self):
            self.get_logger().info('Robot is Turning and Moving Forward')
            self.publisher_.publish(self.spin_msg)
            self.spin_msg.angular.z += 0.15
            self.spin_msg.linear.x -= 0.01

def main(args=None):
    rclpy.init(args=args)
    draw_spiral = DrawSpiral()

    rclpy.spin(draw_spiral)
    draw_spiral.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
