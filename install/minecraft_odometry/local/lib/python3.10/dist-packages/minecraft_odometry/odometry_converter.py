#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import math
from collections import deque
import threading

from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class OdometryConverter(Node):
    def __init__(self):
        super().__init__('odometry_converter')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('odom_frame_id', 'odom'),
                ('base_frame_id', 'base_link'),
                ('child_frame_id', 'base_link'),
                ('publish_rate', 10.0),
                ('position_covariance', [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]),
                ('orientation_covariance', [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]),
                ('linear_velocity_covariance', [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]),
                ('angular_velocity_covariance', [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]),
                ('velocity_rolling_window_size', 5),
                ('transform_tolerance', 0.1),
                ('scale_factor', 1.0)
            ]
        )
        
        # Get parameters
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.position_covariance = self.get_parameter('position_covariance').get_parameter_value().double_array_value
        self.orientation_covariance = self.get_parameter('orientation_covariance').get_parameter_value().double_array_value
        self.linear_velocity_covariance = self.get_parameter('linear_velocity_covariance').get_parameter_value().double_array_value
        self.angular_velocity_covariance = self.get_parameter('angular_velocity_covariance').get_parameter_value().double_array_value
        self.velocity_window_size = self.get_parameter('velocity_rolling_window_size').get_parameter_value().integer_value
        self.transform_tolerance = self.get_parameter('transform_tolerance').get_parameter_value().double_value
        self.scale_factor = self.get_parameter('scale_factor').get_parameter_value().double_value
        
        # Publishers and subscribers
        self.ground_truth_sub = self.create_subscription(
            Pose,
            '/player/ground_truth',
            self.ground_truth_callback,
            10
        )
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # State variables
        self.previous_pose = None
        self.previous_time = None
        self.velocity_history = deque(maxlen=self.velocity_window_size)
        self.origin_pose = None  # First pose will be used as origin
        
        # Time synchronization - use original ground truth timestamps
        self.timestamp_lock = threading.Lock()
        self.last_timestamp = None
        
        # Initialize odometry message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame_id
        self.odom_msg.child_frame_id = self.child_frame_id
        
        # Set covariance matrices
        self.set_covariance_matrices()
        
        self.get_logger().info(f'Odometry converter initialized')
        self.get_logger().info(f'Publishing odometry from {self.odom_frame_id} to {self.child_frame_id}')
        
    def set_covariance_matrices(self):
        """Set covariance matrices for pose and twist"""
        # Position and orientation covariance (6x6 matrix)
        pose_covariance = [0.0] * 36
        # Position covariance (x, y, z)
        for i in range(3):
            if i < len(self.position_covariance):
                pose_covariance[i * 6 + i] = self.position_covariance[i]
        # Orientation covariance (roll, pitch, yaw)
        for i in range(3):
            if i < len(self.orientation_covariance):
                pose_covariance[(i + 3) * 6 + (i + 3)] = self.orientation_covariance[i]
        
        self.odom_msg.pose.covariance = pose_covariance
        
        # Linear and angular velocity covariance (6x6 matrix)
        twist_covariance = [0.0] * 36
        # Linear velocity covariance (x, y, z)
        for i in range(3):
            if i < len(self.linear_velocity_covariance):
                twist_covariance[i * 6 + i] = self.linear_velocity_covariance[i]
        # Angular velocity covariance (roll, pitch, yaw)
        for i in range(3):
            if i < len(self.angular_velocity_covariance):
                twist_covariance[(i + 3) * 6 + (i + 3)] = self.angular_velocity_covariance[i]
        
        self.odom_msg.twist.covariance = twist_covariance
    
    def get_current_time(self):
        """Get current time using the actual message timestamp"""
        # Just use the current ROS time, but this will be overridden in the callback
        return self.get_clock().now()
        
    def ground_truth_callback(self, msg):
        """Convert ground truth pose to odometry using direct calculation"""
        # Use current ROS time to match with sensor_synchronizer
        current_time = self.get_clock().now()
        
        # Set origin from first pose
        if self.origin_pose is None:
            self.origin_pose = Pose()
            self.origin_pose.position.x = msg.position.x
            self.origin_pose.position.y = msg.position.y
            self.origin_pose.position.z = msg.position.z
            self.origin_pose.orientation = msg.orientation
            self.get_logger().info(f'Origin set at: ({msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f})')
            
            # Publish static world -> map transform (origin)
            self.publish_world_to_map_transform()
        
        # Calculate relative position from origin (direct calculation)
        scaled_pose = Pose()
        scaled_pose.position.x = (msg.position.x - self.origin_pose.position.x) * self.scale_factor
        scaled_pose.position.y = (msg.position.y - self.origin_pose.position.y) * self.scale_factor
        scaled_pose.position.z = (msg.position.z - self.origin_pose.position.z) * self.scale_factor
        
        # Calculate relative orientation from origin (전체 3D 회전 고려)
        # 상대적 orientation = current * inverse(origin)
        scaled_pose.orientation = self.calculate_relative_orientation(
            msg.orientation,
            self.origin_pose.orientation
        )
        
        # Update odometry message with current time
        self.odom_msg.header.stamp = current_time.to_msg()
        self.odom_msg.pose.pose = scaled_pose
        
        # Calculate velocity if we have previous data
        if self.previous_pose is not None and self.previous_time is not None:
            dt = (current_time - self.previous_time).nanoseconds / 1e9
            if dt > 0.001:  # Avoid division by very small numbers
                # Calculate linear velocity
                dx = scaled_pose.position.x - self.previous_pose.position.x
                dy = scaled_pose.position.y - self.previous_pose.position.y
                dz = scaled_pose.position.z - self.previous_pose.position.z
                
                linear_vel = [dx / dt, dy / dt, dz / dt]
                
                # Calculate angular velocity (3D quaternion 기반)
                # Angular velocity = 2 * (current_quat * inverse(previous_quat)).xyz / dt
                relative_quat = self.quaternion_multiply(
                    scaled_pose.orientation,
                    self.quaternion_inverse(self.previous_pose.orientation)
                )
                
                # Extract angular velocity from relative quaternion
                # ω = 2 * q.xyz / dt (for small rotations)
                angular_vel = [
                    2.0 * relative_quat.x / dt,
                    2.0 * relative_quat.y / dt,
                    2.0 * relative_quat.z / dt
                ]
                
                # Add to velocity history for smoothing
                self.velocity_history.append((linear_vel, angular_vel))
                
                # Calculate smoothed velocity
                if len(self.velocity_history) > 0:
                    avg_linear = [0.0, 0.0, 0.0]
                    avg_angular = [0.0, 0.0, 0.0]
                    
                    for lin_vel, ang_vel in self.velocity_history:
                        for i in range(3):
                            avg_linear[i] += lin_vel[i]
                            avg_angular[i] += ang_vel[i]
                    
                    count = len(self.velocity_history)
                    for i in range(3):
                        avg_linear[i] /= count
                        avg_angular[i] /= count
                    
                    # Set velocity in odometry message
                    self.odom_msg.twist.twist.linear.x = avg_linear[0]
                    self.odom_msg.twist.twist.linear.y = avg_linear[1]
                    self.odom_msg.twist.twist.linear.z = avg_linear[2]
                    self.odom_msg.twist.twist.angular.x = avg_angular[0]
                    self.odom_msg.twist.twist.angular.y = avg_angular[1]
                    self.odom_msg.twist.twist.angular.z = avg_angular[2]
        
        # Publish odometry
        self.odom_pub.publish(self.odom_msg)
        
        # Publish transform (standard SLAM: odometry publishes odom->base_link)
        self.publish_transform(scaled_pose, current_time)
        
        # Update previous state
        self.previous_pose = scaled_pose
        self.previous_time = current_time
        
    def publish_transform(self, pose, timestamp):
        """Publish odom -> base_link transform"""
        transform = TransformStamped()
        
        # Use the exact same timestamp as odometry message
        transform.header.stamp = timestamp.to_msg()
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.child_frame_id
        
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        
        transform.transform.rotation = pose.orientation
        
        self.tf_broadcaster.sendTransform(transform)
        
    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle"""
        # Convert quaternion to Euler angles (specifically yaw)
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def quaternion_inverse(self, q):
        """Calculate quaternion inverse (conjugate for unit quaternions)"""
        from geometry_msgs.msg import Quaternion
        inv_q = Quaternion()
        inv_q.x = -q.x
        inv_q.y = -q.y
        inv_q.z = -q.z
        inv_q.w = q.w
        return inv_q
    
    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions: q1 * q2"""
        from geometry_msgs.msg import Quaternion
        result = Quaternion()
        result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
        result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        return result
    
    def calculate_relative_orientation(self, current_quat, origin_quat):
        """Calculate relative orientation: current * inverse(origin)"""
        origin_inv = self.quaternion_inverse(origin_quat)
        return self.quaternion_multiply(current_quat, origin_inv)


    def publish_world_to_map_transform(self):
        """Publish static world->map transform at origin pose (ground truth 기준)"""
        static_transform = TransformStamped()
        
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'world'
        static_transform.child_frame_id = 'map'
        
        # Ground truth origin을 map frame의 원점으로 설정 (스케일링 없이)
        # 이 변환은 world 좌표계에서 ground truth origin 위치를 나타냄
        static_transform.transform.translation.x = self.origin_pose.position.x
        static_transform.transform.translation.y = self.origin_pose.position.y
        static_transform.transform.translation.z = self.origin_pose.position.z
        
        # Ground truth의 첫 orientation을 map frame 기준으로 설정 (전체 3D 회전)
        static_transform.transform.rotation = self.origin_pose.orientation
        
        self.static_tf_broadcaster.sendTransform(static_transform)
        self.get_logger().info(f'Published static world->map transform at ground truth origin: ({self.origin_pose.position.x:.2f}, {self.origin_pose.position.y:.2f}, {self.origin_pose.position.z:.2f})')




def main(args=None):
    rclpy.init(args=args)
    
    odometry_converter = OdometryConverter()
    
    try:
        rclpy.spin(odometry_converter)
    except KeyboardInterrupt:
        pass
    finally:
        odometry_converter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()