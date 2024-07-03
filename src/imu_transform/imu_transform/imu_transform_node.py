import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class IMUTransformNode(Node):
    def __init__(self):
        super().__init__('imu_transform_node')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
        self.publisher = self.create_publisher(Imu, '/imu_transformed', 10)

        # Initialize the transformed message
        self.transformed_msg = Imu()

        # Transformation matrix from camera optical (Z forward, X right, Y down) to ENU (X forward, Y left, Z up)
        self.transform = np.array([
            [0, 0, 1],  # Z forward becomes X forward
            [-1, 0, 0], # X right becomes Y left
            [0, -1, 0]  # Y down becomes Z up
        ])

        # Timer to publish at 60 Hz
        self.timer = self.create_timer(1/60, self.publish_transformed_data)

    def imu_callback(self, msg):

        self.transformed_msg.header.stamp = self.get_clock().now().to_msg()
        self.transformed_msg.header.frame_id = 'imu_transformed_optical_frame'

        # Transform angular velocity
        angular_vel = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        transformed_angular_vel = self.transform @ angular_vel
        self.transformed_msg.angular_velocity.x = transformed_angular_vel[0]
        self.transformed_msg.angular_velocity.y = transformed_angular_vel[1]
        self.transformed_msg.angular_velocity.z = transformed_angular_vel[2]

        # Transform linear acceleration
        linear_accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        transformed_linear_accel = self.transform @ linear_accel
        self.transformed_msg.linear_acceleration.x = transformed_linear_accel[0]
        self.transformed_msg.linear_acceleration.y = transformed_linear_accel[1]
        self.transformed_msg.linear_acceleration.z = transformed_linear_accel[2]

        # Covariances (assuming they should be transformed in a similar way; this is a simplification)
        self.transformed_msg.orientation_covariance = msg.orientation_covariance
        self.transformed_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        self.transformed_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

    def publish_transformed_data(self):
        self.publisher.publish(self.transformed_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_transform_node = IMUTransformNode()
    rclpy.spin(imu_transform_node)
    imu_transform_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
