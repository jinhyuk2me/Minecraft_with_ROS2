#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
import rclpy.time
import threading
from collections import deque
import time
from tf2_ros import TransformListener, Buffer
import tf2_sensor_msgs
from rclpy.duration import Duration

class SensorSynchronizerV2(Node):
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
        
        # Message buffering system
        self.message_buffer = []
        self.buffer_lock = threading.Lock()
        self.buffer_size = 20  # Keep last 20 messages
        
        # Timing system
        self.global_counter = 0
        self.last_timestamp = None
        self.counter_lock = threading.Lock()
        
        # TF listener for coordinate transformation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Processing timer - process buffered messages every 10ms
        self.process_timer = self.create_timer(0.01, self.process_buffer)
        
        self.get_logger().info('Sensor synchronizer V2 initialized')
        
    def get_synchronized_timestamp(self, original_timestamp):
        """Get synchronized timestamp using current ROS time"""
        # Use current ROS time to match with odometry_converter그
        with self.counter_lock:
            current_time = self.get_clock().now()
            
            # Ensure timestamp is always increasing by adding small increments
            if self.last_timestamp is None:
                self.last_timestamp = current_time
                return current_time
                
            if current_time <= self.last_timestamp:
                self.last_timestamp = self.last_timestamp + rclpy.time.Duration(nanoseconds=1_000_000)  # +1ms
            else:
                self.last_timestamp = current_time
                
            return self.last_timestamp
    
    def pointcloud_callback(self, msg):
        """Buffer pointcloud message"""
        with self.buffer_lock:
            self.message_buffer.append({
                'type': 'pointcloud',
                'data': msg,
                'arrival_time': time.time()
            })
            # Keep buffer size manageable
            if len(self.message_buffer) > self.buffer_size:
                self.message_buffer.pop(0)
    
    def imu_callback(self, msg):
        """Buffer IMU message"""
        with self.buffer_lock:
            self.message_buffer.append({
                'type': 'imu',
                'data': msg,
                'arrival_time': time.time()
            })
            # Keep buffer size manageable
            if len(self.message_buffer) > self.buffer_size:
                self.message_buffer.pop(0)
    
    def process_buffer(self):
        """Process buffered messages in chronological order"""
        with self.buffer_lock:
            if len(self.message_buffer) == 0:
                return
            
            # Sort messages by arrival time to ensure proper ordering
            sorted_messages = sorted(self.message_buffer, key=lambda x: x['arrival_time'])
            
            # Process all messages
            for msg_entry in sorted_messages:
                if msg_entry['type'] == 'pointcloud':
                    self.process_pointcloud(msg_entry['data'])
                elif msg_entry['type'] == 'imu':
                    self.process_imu(msg_entry['data'])
            
            # Clear processed messages
            self.message_buffer.clear()
    
    def process_pointcloud(self, msg):
        """Process and republish pointcloud transformed to odom frame"""
        sync_time = self.get_synchronized_timestamp(msg.header.stamp)
        
        try:
            # Transform pointcloud from base_link to odom frame
            # This makes the pointcloud fixed in the environment
            transform = self.tf_buffer.lookup_transform(
                'odom',           # target frame (환경 고정)
                'base_link',      # source frame (센서)
                sync_time,        # time
                timeout=Duration(seconds=0.1)
            )
            
            # Create synchronized message with original data but updated timestamp
            temp_msg = PointCloud2()
            temp_msg.header.stamp = sync_time.to_msg()
            temp_msg.header.frame_id = 'base_link'
            temp_msg.height = msg.height
            temp_msg.width = msg.width
            temp_msg.fields = msg.fields
            temp_msg.is_bigendian = msg.is_bigendian
            temp_msg.point_step = msg.point_step
            temp_msg.row_step = msg.row_step
            temp_msg.data = msg.data
            temp_msg.is_dense = msg.is_dense
            
            # Transform to odom frame
            transformed_msg = tf2_sensor_msgs.do_transform_cloud(temp_msg, transform)
            
            # Publish transformed message (now in odom frame)
            self.pointcloud_pub.publish(transformed_msg)
            
        except Exception as e:
            self.get_logger().debug(f'Failed to transform pointcloud: {e}')
            # Fallback: publish original with base_link frame
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
            
            self.pointcloud_pub.publish(sync_msg)
    
    def process_imu(self, msg):
        """Process and republish IMU with synchronized timestamp"""
        sync_time = self.get_synchronized_timestamp(msg.header.stamp)
        
        # Create synchronized message
        sync_msg = Imu()
        sync_msg.header.stamp = sync_time.to_msg()
        sync_msg.header.frame_id = 'base_link'
        
        # Copy IMU data
        sync_msg.orientation = msg.orientation
        sync_msg.angular_velocity = msg.angular_velocity
        sync_msg.linear_acceleration = msg.linear_acceleration
        
        # Set appropriate covariance values for Cartographer
        sync_msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        sync_msg.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]
        
        sync_msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        # Publish synchronized message
        self.imu_pub.publish(sync_msg)

def main(args=None):
    rclpy.init(args=args)
    
    sensor_synchronizer = SensorSynchronizerV2()
    
    try:
        rclpy.spin(sensor_synchronizer)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_synchronizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()