#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
import rclpy.time
import threading

class SensorSynchronizer(Node):
    def __init__(self):
        super().__init__('sensor_synchronizer')
        
        # Subscribers to original sensors
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/player/pointcloud',
            self.pointcloud_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/player/imu',
            self.imu_callback,
            10
        )
        
        # Publishers for synchronized sensors
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/points2',
            10
        )
        
        self.imu_pub = self.create_publisher(
            Imu,
            '/imu',
            10
        )
        
        # Time tracking for synchronization - use ROS time as base for consistent timing
        self.global_counter = 0
        # Use a fixed start time that's the same across all nodes
        current_ros_time = self.get_clock().now()
        # Round down to the nearest second for consistency
        start_seconds = int(current_ros_time.nanoseconds // 1_000_000_000)
        self.start_time = rclpy.time.Time(nanoseconds=start_seconds * 1_000_000_000)
        self.timestamp_lock = threading.Lock()
        
        self.get_logger().info(f'Sensor sync start time: {self.start_time.nanoseconds}')
        
        self.get_logger().info('Sensor synchronizer initialized')
        
    def get_next_timestamp(self):
        """Get next strictly increasing timestamp for any sensor - thread safe"""
        with self.timestamp_lock:
            timestamp = self.start_time + rclpy.time.Duration(nanoseconds=self.global_counter * 5_000_000)  # 5ms intervals
            self.global_counter += 1
            return timestamp
    
    def pointcloud_callback(self, msg):
        """Process and republish pointcloud with synchronized timestamp"""
        # Get next timestamp in sequence
        sync_time = self.get_next_timestamp()
        
        # Create synchronized message
        sync_msg = PointCloud2()
        sync_msg.header.stamp = sync_time.to_msg()
        sync_msg.header.frame_id = 'base_link'
        sync_msg.height = msg.height
        sync_msg.width = msg.width
        sync_msg.fields = msg.fields
        sync_msg.is_bigendian = msg.is_bigendian
        sync_msg.point_step = msg.point_step
        sync_msg.row_step = msg.row_step
        sync_msg.data = msg.data
        sync_msg.is_dense = msg.is_dense
        
        # Publish synchronized message
        self.pointcloud_pub.publish(sync_msg)
        
    def imu_callback(self, msg):
        """Process and republish IMU with synchronized timestamp"""
        # Get next timestamp in sequence  
        sync_time = self.get_next_timestamp()
        
        # Create synchronized message
        sync_msg = Imu()
        sync_msg.header.stamp = sync_time.to_msg()
        sync_msg.header.frame_id = 'base_link'  # Keep player frame since tracking_frame is now player
        
        # Copy IMU data
        sync_msg.orientation = msg.orientation
        sync_msg.angular_velocity = msg.angular_velocity
        sync_msg.linear_acceleration = msg.linear_acceleration
        
        # Set appropriate covariance values for Cartographer
        # Orientation covariance (3x3 matrix flattened to 9 elements)
        sync_msg.orientation_covariance = [
            0.01, 0.0, 0.0,  # Roll variance
            0.0, 0.01, 0.0,  # Pitch variance  
            0.0, 0.0, 0.01   # Yaw variance
        ]
        
        # Angular velocity covariance
        sync_msg.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]
        
        # Linear acceleration covariance
        sync_msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        # Publish synchronized message
        self.imu_pub.publish(sync_msg)

def main(args=None):
    rclpy.init(args=args)
    
    sensor_synchronizer = SensorSynchronizer()
    
    try:
        rclpy.spin(sensor_synchronizer)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_synchronizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()