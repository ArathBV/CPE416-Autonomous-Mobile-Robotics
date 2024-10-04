import rclpy
import math
from rclpy.node import Node

from std_msgs.msg import Int32

class squared(Node):

    def __init__(self):
        super().__init__('squared')

        self.subscription = self.create_subscription(
                Int32,
                'numbers',
                self.listener_callback,
                10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%d"' % (msg.data ** 2))

def main(args=None):
    rclpy.init(args=args)

    sub = squared()
    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
