import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist

import tf2_ros
import tf2_geometry_msgs
from tf_transformations import translation_matrix, quaternion_matrix, quaternion_from_matrix
from tf2_ros import TransformException

import numpy as np

class bumpNgo(Node):

    def __init__(self):
        super().__init__('bumpNgo')

        # Distance Measurer
        self.scan_sub = self.create_subscription(
            LaserScan, 
            'diff_drive/scan',
            self.scan_callback, 
            10
        )

        # FSM States
        self.stateCurr = "FORWARD"
        self.state0 = "FORWARD"
        self.state1 = "STOP"
        self.state2 = "BACKWARD"
        self.state3 = "FAILED"
        self.state4 = "TURN"

        # Timer for FSM callback function
        self.publisher_ = self.create_publisher(
            Twist,
            'diff_drive/cmd_vel',
            10)

        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timertick = 0

        # Forward and Turning Messages
        self.turning = False
        self.forward_msg = Twist()
        self.turn_msg = Twist()
        self.turns = 0

        #ROS2 Transformation tree and Odom->Object
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.latest_laser = LaserScan()
        return

    def transform_to_matrix(self, transform):
        # Convert a Transform message to a 4x4 transformation matrix
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        rotation = np.array(quaternion_matrix((
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ))[:3, :3])  # Extract the rotation part

        # Create the full transformation matrix
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        matrix[:3, 3] = translation
        return matrix

    def multiply_transforms(self, transform_a, transform_b):
        # Multiply two Transform messages and return the resulting Transform
        # Convert transforms to matrices
        matrix_a = self.transform_to_matrix(transform_a)
        matrix_b = self.transform_to_matrix(transform_b)

        # Perform matrix multiplication
        result_matrix = np.dot(matrix_a, matrix_b)

        # Extract translation and rotation from the result matrix
        result_transform = TransformStamped()
        result_transform.transform.translation.x = result_matrix[0, 3]
        result_transform.transform.translation.y = result_matrix[1, 3]
        result_transform.transform.translation.z = result_matrix[2, 3]
        
        # Convert the rotation part back to a quaternion
        result_rotation = quaternion_from_matrix(result_matrix)
        result_transform.transform.rotation.x = result_rotation[0]
        result_transform.transform.rotation.y = result_rotation[1]
        result_transform.transform.rotation.z = result_rotation[2]
        result_transform.transform.rotation.w = result_rotation[3]

        return result_transform
    
    def process_scan(self):
        # Do nothing if there is no data from the laser yet
        if not self.latest_laser.ranges:
            return

        # Here:
        # Access the laser that is aligned with x-axis of the robot
        # Put it in a variable called 'distance'
        distance = self.latest_laser.ranges[len(self.latest_laser.ranges) // 2]

        if not np.isinf(distance):

            laser2object_msg = TransformStamped()
            # What is the transformation from the Lidar --> Object?
            # Think in terms of 2D displacement
            laser2object_msg.transform.translation.x = distance
            laser2object_msg.transform.translation.y = 0.0
            laser2object_msg.transform.translation.z = 0.0
            laser2object_msg.transform.rotation.x = 0.0
            laser2object_msg.transform.rotation.y = 0.0
            laser2object_msg.transform.rotation.z = 0.0
            laser2object_msg.transform.rotation.w = 1.0            

            laser2object_msg.header.frame_id = self.latest_laser.header.frame_id # Set the correct parent frame
            laser2object_msg.child_frame_id = 'detected_obstacle' # Set correct child frame

            try:
                # Lookup transform
                transform = self.tf_buffer.lookup_transform('diff_drive/odom', self.latest_laser.header.frame_id, rclpy.time.Time())

            except TransformException as ex:
                self.get_logger().warn(f'Obstacle transform not found: {ex}')
                self.stateCurr = self.state1
                return

            # Its usually best not to work with the gemoetry_msgs message types directly
            # since they have limited functionality. In this example, I have provided a some
            # functions to work with the robot: 'transform_to_matrix', 'multiply_transforms'

            # What order should the two transformations be multiplied?
            odom2object_msg = self.multiply_transforms(transform, laser2object_msg)

            odom2object_msg.header.frame_id = 'diff_drive/odom' # Set correct parent frame
            odom2object_msg.child_frame_id = 'detected_obstacle' # Set correct child frame

            self.tf_broadcaster.sendTransform(odom2object_msg)

    def scan_callback(self):
        if self.stateCurr == "FORWARD":
            self.forward_msg.linear.x = 1.0
            distance = self.latest_laser.ranges[len(self.latest_laser.ranges) // 2]
            # Change to Backwards Distance
            if distance == 0.0:
                self.get_logger().info('Robot Bumped into Object. State = Backward')
                self.stateCurr = self.state2
            self.get_logger().info('Robot Moving Forward. State = Forward')

        elif self.stateCurr == "BACKWARD":
            self.timertick += 1
            self.forward_msg.linear.x = -1.0
            if self.timertick % 2 == 0:
                self.timertick = 0
                self.forward_msg.linear.x = 0.0
                self.get_logger().info("Robot Moved Backward for 2 sec. State = Turn")
                self.stateCurr = self.state4
            self.get_logger().info("Robot Moving Backwards. State = Backward")

        elif self.stateCurr == "STOP":
            self.get_logger().info("Robot error occured, unable to move. State = Stop")
            return
        elif self.stateCurr == "TURN":
            # Total of 360/45 = 8 turns to check
            # 45 degree turns = 0.707 rad/s
            distance = self.latest_laser.ranges[len(self.latest_laser.ranges) // 2]
            if distance < 4.0:
                if self.turns == 8:
                    self.stateCurr = self.state1
                    self.get_logger().info("Robot unable to proceed. State = Stop")
                    self.turns = 0
                else:
                    self.turn_msg.angular.z = 0.707
                    self.get_logger().info("Robot turns 45 deg to check for Clearance. State = Turn")
                    self.turns += 1
            else:
                self.get_logger().info("Robot found clearance. State = Forward")
                self.stateCurr = self.state0
        
def main(args=None):
    rclpy.init(args=args)
    node = bumpNgo()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
