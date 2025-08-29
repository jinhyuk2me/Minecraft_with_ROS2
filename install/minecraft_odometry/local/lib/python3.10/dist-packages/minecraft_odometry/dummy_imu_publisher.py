#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class DummyImuPublisher(Node):
    def __init__(self):
        super().__init__('dummy_imu_publisher')
        
        # Publisher for IMU data
        self.publisher = self.create_publisher(Imu, '/imu', 10)
        
        # Timer to publish IMU data
        self.timer = self.create_timer(0.01, self.publish_imu)  # 100Hz
        
        self.get_logger().info('Dummy IMU publisher initialized')
        
    def publish_imu(self):
        """Publish dummy IMU data with zero motion"""
        msg = Imu()
        
        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Orientation (identity quaternion - no rotation)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        
        # Angular velocity (no rotation)
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        
        # Linear acceleration (gravity only)
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81  # gravity
        
        # Covariance matrices (high uncertainty)
        msg.orientation_covariance = [0.1] * 9
        msg.angular_velocity_covariance = [0.1] * 9
        msg.linear_acceleration_covariance = [0.1] * 9
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    dummy_imu = DummyImuPublisher()
    
    try:
        rclpy.spin(dummy_imu)
    except KeyboardInterrupt:
        pass
    finally:
        dummy_imu.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()