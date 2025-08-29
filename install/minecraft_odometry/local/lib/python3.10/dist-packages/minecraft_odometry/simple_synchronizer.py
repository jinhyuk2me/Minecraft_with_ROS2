#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
import threading

class SimpleSynchronizer(Node):
    def __init__(self):
        super().__init__('simple_synchronizer')
        
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
        
        # Simple timestamp management - just ensure increasing order
        self.last_timestamp = None
        self.timestamp_lock = threading.Lock()
        
        self.get_logger().info('Simple synchronizer initialized - minimal timestamp ordering')
        
    def get_next_timestamp(self, original_timestamp):
        """Ensure timestamps are always increasing"""
        with self.timestamp_lock:
            current_time = rclpy.time.Time.from_msg(original_timestamp)
            
            if self.last_timestamp is None:
                self.last_timestamp = current_time
                return current_time
            
            # If timestamp goes backwards, add small increment to last timestamp
            if current_time <= self.last_timestamp:
                self.last_timestamp = self.last_timestamp + rclpy.time.Duration(nanoseconds=1_000_000)  # +1ms
            else:
                self.last_timestamp = current_time
                
            return self.last_timestamp
    
    def pointcloud_callback(self, msg):
        """Republish pointcloud with corrected timestamp"""
        sync_time = self.get_next_timestamp(msg.header.stamp)
        
        # Keep original message, just fix timestamp and frame
        sync_msg = PointCloud2()
        sync_msg.header.stamp = sync_time.to_msg()
        sync_msg.header.frame_id = 'player'  # Keep original frame
        sync_msg.height = msg.height
        sync_msg.width = msg.width
        sync_msg.fields = msg.fields
        sync_msg.is_bigendian = msg.is_bigendian
        sync_msg.point_step = msg.point_step
        sync_msg.row_step = msg.row_step
        sync_msg.data = msg.data
        sync_msg.is_dense = msg.is_dense
        
        self.pointcloud_pub.publish(sync_msg)
        
    def imu_callback(self, msg):
        """Republish IMU with corrected timestamp"""
        sync_time = self.get_next_timestamp(msg.header.stamp)
        
        # Keep original message, just fix timestamp and frame
        sync_msg = Imu()
        sync_msg.header.stamp = sync_time.to_msg()
        sync_msg.header.frame_id = 'player'  # Keep original frame
        
        # Copy all IMU data as-is
        sync_msg.orientation = msg.orientation
        sync_msg.angular_velocity = msg.angular_velocity
        sync_msg.linear_acceleration = msg.linear_acceleration
        sync_msg.orientation_covariance = msg.orientation_covariance
        sync_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        sync_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        
        self.imu_pub.publish(sync_msg)

def main(args=None):
    rclpy.init(args=args)
    
    simple_synchronizer = SimpleSynchronizer()
    
    try:
        rclpy.spin(simple_synchronizer)
    except KeyboardInterrupt:
        pass
    finally:
        simple_synchronizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
