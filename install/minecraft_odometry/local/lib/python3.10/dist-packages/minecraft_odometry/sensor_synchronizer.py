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
import struct

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
        """Process and republish pointcloud in base_link frame for SLAM"""
        sync_time = self.get_synchronized_timestamp(msg.header.stamp)
        
        # Transform pointcloud from Minecraft coordinate system to ROS coordinate system
        # Minecraft: X=East/West, Y=Up/Down, Z=North/South
        # ROS: X=Forward/Back, Y=Left/Right, Z=Up/Down
        # Transformation: Minecraft(X,Y,Z) -> ROS(Z,X,Y)
        transformed_data = self.transform_pointcloud_coordinates(msg.data, msg.point_step)
        
        sync_msg = PointCloud2()
        sync_msg.header.stamp = sync_time.to_msg()
        sync_msg.header.frame_id = 'base_link'  # Keep in sensor frame for SLAM
        sync_msg.height = msg.height
        sync_msg.width = msg.width
        sync_msg.fields = msg.fields
        sync_msg.is_bigendian = msg.is_bigendian
        sync_msg.point_step = msg.point_step
        sync_msg.row_step = msg.row_step
        sync_msg.data = transformed_data
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
    
    def transform_pointcloud_coordinates(self, data, point_step):
        """Transform pointcloud coordinates from Minecraft to ROS coordinate system"""
        # PointCloud2 data format: x(float32), y(float32), z(float32), rgb(float32)
        # Each point is 16 bytes (4 floats)
        
        if point_step != 16:
            self.get_logger().warn(f"Unexpected point_step: {point_step}, expected 16")
            return data
        
        num_points = len(data) // point_step
        transformed_data = []
        
        for i in range(num_points):
            offset = i * point_step
            point_bytes = data[offset:offset + point_step]
            
            # Unpack: x, y, z, rgb
            x, y, z, rgb = struct.unpack('<ffff', bytes(point_bytes))
            
            # Coordinate transformation: Minecraft(x,y,z) -> ROS(z,x,y)
            # Minecraft X (East/West) -> ROS Y (Left/Right)
            # Minecraft Y (Up/Down) -> ROS Z (Up/Down)  
            # Minecraft Z (North/South) -> ROS X (Forward/Back)
            ros_x = z
            ros_y = x
            ros_z = y
            
            # Pack back to bytes
            transformed_point = struct.pack('<ffff', ros_x, ros_y, ros_z, rgb)
            transformed_data.extend(transformed_point)
        
        return transformed_data

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