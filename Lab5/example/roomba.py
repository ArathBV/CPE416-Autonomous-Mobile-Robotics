import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf2_ros import TransformException, Buffer, TransformListener, StaticTransformBroadcaster

class bumpNgo(Node):

    def __init__(self):
        super().__init__('bumpNgo')

        # Subscribe to LaserScan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            'diff_drive/scan',
            self.scan_callback,
            10
        )

        # Publisher for Twist commands
        self.publisher_ = self.create_publisher(Twist, 'diff_drive/cmd_vel', 10)

        # Timer for FSM callback function
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Initialize FSM States
        self.stateCurr = "FORWARD"
        self.states = {"FORWARD": 0, "STOP": 1, "BACKWARD": 2, "FAILED": 3, "TURN": 4}
        self.timertick = 0

        # Movement Commands
        self.forward_msg = Twist()
        self.turn_msg = Twist()
        self.turns = 0

        # Transformation setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # LaserScan data placeholder
        self.latest_laser = None

    def scan_callback(self, msg):
        # Update latest laser data
        self.latest_laser = msg

    def timer_callback(self):
        # Check LaserScan data
        if self.latest_laser is None or not self.latest_laser.ranges:
            self.get_logger().error("LaserScan failed. Ending program.")
            self.stateCurr = "FAILED"
            return

        # FSM Handling
        if self.stateCurr == "FORWARD":
            distance = self.latest_laser.ranges[len(self.latest_laser.ranges) // 2]
            if distance > 0.0:
                self.forward_msg.linear.x = 1.0
                self.publisher_.publish(self.forward_msg)
                self.get_logger().info("Robot Moving Forward. State = Forward")
            else:
                self.get_logger().info("Bumped into Object. State = Backward")
                self.stateCurr = "BACKWARD"

        elif self.stateCurr == "BACKWARD":
            self.timertick += 1
            self.forward_msg.linear.x = -1.0
            self.publisher_.publish(self.forward_msg)
            if self.timertick % 2 == 0:
                self.timertick = 0
                self.forward_msg.linear.x = 0.0
                self.publisher_.publish(self.forward_msg)
                self.get_logger().info("Moved Backward. State = Turn")
                self.stateCurr = "TURN"

        elif self.stateCurr == "TURN":
            # Check for clearance with turns
            distance = self.latest_laser.ranges[len(self.latest_laser.ranges) // 2]
            if distance < 4.0:
                if self.turns >= 8:
                    self.get_logger().info("Obstacle in all directions. State = Stop")
                    self.stateCurr = "STOP"
                    self.turns = 0
                else:
                    self.turn_msg.angular.z = 0.707
                    self.publisher_.publish(self.turn_msg)
                    self.get_logger().info("Turning 45 degrees. State = Turn")
                    self.turns += 1
            else:
                self.get_logger().info("Found clearance. State = Forward")
                self.stateCurr = "FORWARD"
                self.turns = 0

        elif self.stateCurr == "STOP":
            self.get_logger().info("Robot stopped due to obstacles.")
            self.forward_msg.linear.x = 0.0
            self.publisher_.publish(self.forward_msg)
        
        elif self.stateCurr == "FAILED":
            self.get_logger().error("LaserScan failure detected. Exiting program.")
            self.forward_msg.linear.x = 0.0
            self.publisher_.publish(self.forward_msg)
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = bumpNgo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
