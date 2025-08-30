#!/usr/bin/env python3

# Minecraft에서 받은 ground truth pose를 ROS2 odometry와 TF transform으로 변환하는 노드

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
    """
    Minecraft 플레이어의 ground truth pose를 ROS2 표준 odometry로 변환하는 클래스
    
    주요 기능:
    - /player/ground_truth 토픽을 구독하여 절대 좌표계 pose를 받음
    - 첫 번째 pose를 원점으로 설정하여 상대 좌표계로 변환
    - /odom 토픽으로 odometry 메시지 발행
    - 표준 ROS TF 트리 구성: world → map → odom → base_link
    
    좌표계 정의:
    - Minecraft: X(동쪽+), Y(위+), Z(남쪽+)
    - ROS: X(앞+), Y(왼쪽+), Z(위+)
    
    중요 설계 결정:
    - world→map: 위치만 변환, 방향은 identity (고리 형태 문제 방지)
    - map→odom: identity transform (ground truth 완벽함)
    - odom→base_link: 상대방향 기본 사용 (일관성 있는 좌표계)
    """
    def __init__(self):
        super().__init__('odometry_converter')
        
        # 노드 매개변수 선언 - 런타임에 변경 가능한 설정값들
        self.declare_parameters(
            namespace='',
            parameters=[
                ('odom_frame_id', 'odom'),           # odometry frame ID
                ('base_frame_id', 'base_link'),      # 로봇의 기본 좌표계 이름
                ('child_frame_id', 'base_link'),     # transform의 자식 frame ID
                ('publish_rate', 10.0),              # 발행 주기 (Hz)

                ('position_covariance', [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]),        # 위치 불확실성
                ('orientation_covariance', [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]),     # 방향 불확실성

                ('linear_velocity_covariance', [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]), # 선속도 불확실성
                ('angular_velocity_covariance', [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]),# 각속도 불확실성

                ('velocity_rolling_window_size', 5), # 속도 계산을 위한 윈도우 크기
                ('transform_tolerance', 0.1),        # transform 허용 시간 차이
                ('scale_factor', 1.0),               # 좌표계 크기 조절 팩터
                ('use_relative_orientation', True),  # 상대방향 사용 여부 - base_link는 플레이어 기준이어야 함
                ('use_minecraft_axis_conversion', True), # Minecraft 좌표계를 ROS 좌표계로 변환 여부
                ('yaw_conversion_mode', 3)               # Yaw 변환 모드 - Mode 3이 올바른 ROS 표준
            ]
        )

        # 매개변수 값 읽기 - 선언된 매개변수들을 인스턴스 변수로 저장
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
        self.use_relative_orientation = self.get_parameter('use_relative_orientation').get_parameter_value().bool_value
        self.use_minecraft_axis_conversion = self.get_parameter('use_minecraft_axis_conversion').get_parameter_value().bool_value
        self.yaw_conversion_mode = self.get_parameter('yaw_conversion_mode').get_parameter_value().integer_value
        
        # Publisher와 Subscriber 초기화
        self.ground_truth_sub = self.create_subscription(
            Pose,
            '/player/ground_truth',    # Minecraft에서 오는 플레이어 절대 위치
            self.ground_truth_callback,
            10
        )
        
        self.odom_pub = self.create_publisher(
            Odometry, 
            '/odom', # ROS2 표준 odometry 토픽
            10
        )  
        self.tf_broadcaster = TransformBroadcaster(self)             # 동적 TF 브로드캐스터
        self.static_tf_broadcaster = StaticTransformBroadcaster(self) # 정적 TF 브로드캐스터
        
        # 상태 변수들 - 속도 계산과 변환을 위한 내부 상태 저장
        self.previous_pose = None          # 이전 pose (속도 계산용)
        self.previous_time = None          # 이전 시간 (속도 계산용)
        self.velocity_history = deque(maxlen=self.velocity_window_size)  # 속도 이력 (평균화용)
        self.origin_pose = None            # 첫 번째 pose를 원점으로 설정
        
        # 시간 동기화 - ground truth 메시지의 원본 타임스탬프 사용
        self.timestamp_lock = threading.Lock()
        self.last_timestamp = None
        
        # odometry 메시지 초기화 - 매번 새로 생성하지 않고 재사용
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame_id
        self.odom_msg.child_frame_id = self.child_frame_id
        
        # 공분산 행렬 설정 - 센서 불확실성 정의
        self.set_covariance_matrices()
        
        self.get_logger().info(f'Odometry 변환기가 초기화되었습니다')
        self.get_logger().info(f'{self.odom_frame_id}에서 {self.child_frame_id}로 odometry를 발행합니다')
        orientation_mode = "상대방향" if self.use_relative_orientation else "절대방향"
        axis_mode = "Minecraft→ROS 변환" if self.use_minecraft_axis_conversion else "축 변환 없음"
        self.get_logger().info(f'방향 모드: {orientation_mode} (use_relative_orientation={self.use_relative_orientation})')
        self.get_logger().info(f'좌표축 모드: {axis_mode} (use_minecraft_axis_conversion={self.use_minecraft_axis_conversion})')
        self.get_logger().info(f'Yaw 변환 모드: {self.yaw_conversion_mode} (1=반전, 2=+90°, 3=-90°, 4=+180°)')
        
    def set_covariance_matrices(self):
        """pose와 twist에 대한 공분산 행렬 설정"""
        # 위치와 방향 공분산 행렬 (6x6 행렬 - x,y,z,roll,pitch,yaw)
        pose_covariance = [0.0] * 36
        
        # 위치 공분산 (x, y, z) - 대각선 원소만 설정
        for i in range(3):
            if i < len(self.position_covariance):
                pose_covariance[i * 6 + i] = self.position_covariance[i]
                
        # 방향 공분산 (roll, pitch, yaw) - 대각선 원소만 설정
        for i in range(3):
            if i < len(self.orientation_covariance):
                pose_covariance[(i + 3) * 6 + (i + 3)] = self.orientation_covariance[i]
        
        self.odom_msg.pose.covariance = pose_covariance
        
        # 선속도와 각속도 공분산 행렬 (6x6 행렬)
        twist_covariance = [0.0] * 36
        
        # 선속도 공분산 (x, y, z) - 대각선 원소만 설정
        for i in range(3):
            if i < len(self.linear_velocity_covariance):
                twist_covariance[i * 6 + i] = self.linear_velocity_covariance[i]
                
        # 각속도 공분산 (roll, pitch, yaw) - 대각선 원소만 설정
        for i in range(3):
            if i < len(self.angular_velocity_covariance):
                twist_covariance[(i + 3) * 6 + (i + 3)] = self.angular_velocity_covariance[i]
        
        self.odom_msg.twist.covariance = twist_covariance
    
    def get_current_time(self):
        """현재 시간 반환 - 실제 메시지 타임스탬프 사용"""
        # 현재 ROS 시간 사용, 콜백에서 오버라이드됨
        return self.get_clock().now()
        
    def ground_truth_callback(self, msg):
        """
        Ground truth pose를 TF transform으로 직접 변환하는 콜백 함수
        - 첫 번째 pose를 원점으로 설정
        - 상대적 위치 계산
        - TF transform과 odometry 메시지 발행
        """
        # sensor_synchronizer와 동기화를 위해 현재 ROS 시간 사용
        current_time = self.get_clock().now()
        
        # 첫 번째 pose를 원점으로 설정 - 상대 좌표계의 기준점
        if self.origin_pose is None:
            self.origin_pose = Pose()
            self.origin_pose.position.x = msg.position.x
            self.origin_pose.position.y = msg.position.y
            self.origin_pose.position.z = msg.position.z
            
            # 원점 방향도 좌표축 변환 적용 (상대방향 계산에 필요)
            current_use_conversion = self.get_parameter('use_minecraft_axis_conversion').get_parameter_value().bool_value
            if current_use_conversion:
                self.origin_pose.orientation = self.convert_minecraft_to_ros_orientation(msg.orientation)
                self.get_logger().info(f'원점이 설정되었습니다 (좌표축 변환됨): ({msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f})')
            else:
                self.origin_pose.orientation = msg.orientation
                self.get_logger().info(f'원점이 설정되었습니다 (변환 안됨): ({msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f})')
            
            # 정적 world -> map transform 발행 (첫 pose 기준)
            self.publish_world_to_map_transform()
            
            # 정적 map -> odom transform 발행 - Cartographer TF 발행 완전 비활성화됨
            self.publish_map_to_odom_transform()
            self.get_logger().info('TF 구조 완료: world→map→odom→base_link, Cartographer는 맵만 생성')
        
        # 원점을 기준으로 한 상대적 pose 계산 (현재 구현)
        direct_pose = Pose()
        # 위치 변환: (현재위치 - 원점위치) * 스케일팩터
        direct_pose.position.x = (msg.position.x - self.origin_pose.position.x) * self.scale_factor
        direct_pose.position.y = (msg.position.y - self.origin_pose.position.y) * self.scale_factor
        direct_pose.position.z = (msg.position.z - self.origin_pose.position.z) * self.scale_factor
        
        # 방향 설정 - 절대방향 vs 상대방향 선택 가능 (실시간 매개변수 읽기)
        current_use_relative = self.get_parameter('use_relative_orientation').get_parameter_value().bool_value
        if current_use_relative:
            # 상대방향 사용: 원점 기준으로 한 상대적 회전
            # 예: 원점에서 북쪽(0°)을 보고 있었다면, 현재 동쪽을 보면 +90° 회전으로 표현
            
            # 현재 방향도 좌표축 변환 적용 (원점과 동일한 좌표계에서 계산)
            current_use_conversion = self.get_parameter('use_minecraft_axis_conversion').get_parameter_value().bool_value
            if current_use_conversion:
                current_orientation = self.convert_minecraft_to_ros_orientation(msg.orientation)
            else:
                current_orientation = msg.orientation
            
            relative_orientation = self.calculate_relative_orientation(
                current_orientation,          # 현재 방향 (변환됨)
                self.origin_pose.orientation  # 원점 방향 (이미 변환됨)
            )
            direct_pose.orientation = relative_orientation
            
            # 상대방향의 장점:
            # - odom 좌표계가 항상 원점 방향을 기준으로 함
            # - 로봇이 시작할 때의 방향이 항상 "앞"이 됨
            # - Navigation에서 일관된 방향 기준 제공
            
        else:
            # 절대방향 사용: Minecraft 월드 좌표계의 방향 사용
            current_use_conversion = self.get_parameter('use_minecraft_axis_conversion').get_parameter_value().bool_value
            if current_use_conversion:
                # Minecraft 좌표계를 ROS 좌표계로 변환 (Y축 회전 → Z축 회전)
                direct_pose.orientation = self.convert_minecraft_to_ros_orientation(msg.orientation)
                # 변환 후 장점:
                # - ROS 표준 좌표계 준수 (Z축이 yaw)
                # - rviz에서 올바른 방향 표시
                # - Navigation Stack과 완전 호환
            else:
                # Minecraft 방향을 그대로 사용 (디버깅용)
                direct_pose.orientation.x = msg.orientation.x
                direct_pose.orientation.y = msg.orientation.y
                direct_pose.orientation.z = msg.orientation.z
                direct_pose.orientation.w = msg.orientation.w
                
            # 절대방향의 장점:
            # - 전역 좌표계 기준으로 일관된 방향
            # - 여러 로봇이 있을 때 동일한 방향 기준
        
        # 디버깅을 위한 방향 정보 로그 (10회에 1번만 출력)
        if not hasattr(self, 'debug_counter'):
            self.debug_counter = 0
        self.debug_counter += 1
        
        if self.debug_counter % 10 == 0:  # 10번에 1번만 로그 출력
            current_yaw = self.quaternion_to_yaw(direct_pose.orientation)
            minecraft_yaw = self.quaternion_to_yaw(msg.orientation)
            current_use_conversion = self.get_parameter('use_minecraft_axis_conversion').get_parameter_value().bool_value
            conversion_status = "축 변환됨" if current_use_conversion else "변환 안됨"
            
            # 각도를 도단위로도 표시
            minecraft_yaw_deg = math.degrees(minecraft_yaw)
            current_yaw_deg = math.degrees(current_yaw)
            
            # 실시간 매개변수 읽기
            current_mode = self.get_parameter('yaw_conversion_mode').get_parameter_value().integer_value
            
            self.get_logger().info(f'[DEBUG] === 좌표계 변환 디버그 ===')
            self.get_logger().info(f'[DEBUG] 변환 모드: {current_mode} ({conversion_status})')
            self.get_logger().info(f'[DEBUG] Minecraft yaw: {minecraft_yaw:.3f}rad ({minecraft_yaw_deg:.1f}°)')
            self.get_logger().info(f'[DEBUG] ROS odom yaw: {current_yaw:.3f}rad ({current_yaw_deg:.1f}°)')
            self.get_logger().info(f'[DEBUG] 변환 차이: {current_yaw_deg - minecraft_yaw_deg:.1f}°')
            self.get_logger().info(f'[DEBUG] Minecraft quat: x={msg.orientation.x:.3f}, y={msg.orientation.y:.3f}, z={msg.orientation.z:.3f}, w={msg.orientation.w:.3f}')
            self.get_logger().info(f'[DEBUG] ROS quat: x={direct_pose.orientation.x:.3f}, y={direct_pose.orientation.y:.3f}, z={direct_pose.orientation.z:.3f}, w={direct_pose.orientation.w:.3f}')
            
            # 예상되는 방향 설명
            if current_use_conversion:
                minecraft_direction = self.get_direction_description(minecraft_yaw)
                ros_direction = self.get_direction_description(current_yaw)
                self.get_logger().info(f'[DEBUG] Minecraft 방향: {minecraft_direction} → ROS 방향: {ros_direction}')
            
            self.get_logger().info(f'[DEBUG] ==========================')
        
        # odom->base_link transform 발행 (직접 변환 방식)
        self.publish_transform(direct_pose, current_time)
        
        # 호환성을 위한 odometry 메시지 발행
        self.odom_msg.header.stamp = current_time.to_msg()
        self.odom_msg.pose.pose = direct_pose
        
        # 현재는 pose에만 집중하여 속도를 0으로 설정
        # TODO: 이전 pose와 비교하여 실제 속도 계산 가능
        self.odom_msg.twist.twist.linear.x = 0.0
        self.odom_msg.twist.twist.linear.y = 0.0
        self.odom_msg.twist.twist.linear.z = 0.0
        self.odom_msg.twist.twist.angular.x = 0.0
        self.odom_msg.twist.twist.angular.y = 0.0
        self.odom_msg.twist.twist.angular.z = 0.0
        
        self.odom_pub.publish(self.odom_msg)
        
    def publish_transform(self, pose, timestamp):
        """odom -> base_link TF transform 발행"""
        transform = TransformStamped()
        
        # odometry 메시지와 동일한 타임스탬프 사용 (시간 동기화 중요)
        transform.header.stamp = timestamp.to_msg()
        transform.header.frame_id = self.odom_frame_id      # 부모 프레임: odom
        transform.child_frame_id = self.child_frame_id      # 자식 프레임: base_link
        
        # 위치 변환 설정
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        
        # 방향(quaternion) 변환 설정
        transform.transform.rotation = pose.orientation
        
        # TF 브로드캐스터를 통해 transform 발행
        self.tf_broadcaster.sendTransform(transform)
        
    def quaternion_to_yaw(self, quaternion):
        """Quaternion을 yaw 각도로 변환 (Z축 회전각)"""
        # Quaternion을 오일러 각도로 변환 (특히 yaw)
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def normalize_angle(self, angle):
        """각도를 [-pi, pi] 범위로 정규화"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def quaternion_inverse(self, q):
        """Quaternion의 역원 계산 (단위 quaternion의 경우 켤레복소수)"""
        from geometry_msgs.msg import Quaternion
        inv_q = Quaternion()
        inv_q.x = -q.x  # 허수부의 부호 반전
        inv_q.y = -q.y
        inv_q.z = -q.z
        inv_q.w = q.w   # 실수부는 유지
        return inv_q
    
    def quaternion_multiply(self, q1, q2):
        """두 Quaternion의 곱셈: q1 * q2"""
        from geometry_msgs.msg import Quaternion
        result = Quaternion()
        # Quaternion 곱셈 공식 적용
        result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
        result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        return result
    
    def calculate_relative_orientation(self, current_quat, origin_quat):
        """상대 방향 계산: inverse(origin) * current"""
        origin_inv = self.quaternion_inverse(origin_quat)
        return self.quaternion_multiply(origin_inv, current_quat)
    
    def convert_minecraft_to_ros_orientation(self, minecraft_quat):
        """
        Minecraft 좌표계의 quaternion을 ROS 좌표계로 변환
        
        좌표계 정의:
        Minecraft: X(동쪽+), Y(위+), Z(남쪽+), Yaw는 Y축 중심
        ROS: X(앞+), Y(왼쪽+), Z(위+), Yaw는 Z축 중심
        
        변환 매핑:
        - Minecraft 북쪽(-Z) → ROS 앞(+X)  
        - Minecraft 동쪽(+X) → ROS 왼쪽(+Y)
        - Minecraft 위(+Y) → ROS 위(+Z)
        """
        from geometry_msgs.msg import Quaternion
        import math
        
        # Minecraft quaternion을 yaw 각도로 변환
        minecraft_yaw = self.quaternion_to_yaw(minecraft_quat)
        
        # Minecraft yaw을 ROS yaw으로 변환
        # Minecraft: 북쪽=0°, 동쪽=-90°, 남쪽=±180°, 서쪽=+90° (Y축 중심, 시계방향)
        # ROS: 앞=0°, 왼쪽=+90°, 뒤=±180°, 오른쪽=-90° (Z축 중심, 반시계방향)
        
        # 실시간 테스트를 위한 여러 변환 모드 (매번 매개변수 읽기)
        current_mode = self.get_parameter('yaw_conversion_mode').get_parameter_value().integer_value
        
        if current_mode == 1:
            ros_yaw = -minecraft_yaw  # 방향 반전
        elif current_mode == 2:
            ros_yaw = minecraft_yaw + math.pi/2  # +90도
        elif current_mode == 3:
            ros_yaw = minecraft_yaw - math.pi/2  # -90도
        elif current_mode == 4:
            ros_yaw = minecraft_yaw + math.pi    # +180도
        else:
            ros_yaw = minecraft_yaw  # 변환 없음 (기본값)
        
        # ROS yaw를 quaternion으로 변환 (Z축 회전)
        ros_quat = Quaternion()
        ros_quat.x = 0.0
        ros_quat.y = 0.0
        ros_quat.z = math.sin(ros_yaw / 2.0)
        ros_quat.w = math.cos(ros_yaw / 2.0)
        
        return ros_quat
    
    def get_direction_description(self, yaw_rad):
        """Yaw 각도를 방향 설명으로 변환"""
        import math
        
        # 각도를 0-2π 범위로 정규화
        yaw_normalized = yaw_rad % (2 * math.pi)
        
        # 8방향으로 분류
        if yaw_normalized < math.pi/8 or yaw_normalized >= 15*math.pi/8:
            return "앞(0°)"
        elif yaw_normalized < 3*math.pi/8:
            return "앞-왼쪽(45°)"
        elif yaw_normalized < 5*math.pi/8:
            return "왼쪽(90°)"
        elif yaw_normalized < 7*math.pi/8:
            return "뒤-왼쪽(135°)"
        elif yaw_normalized < 9*math.pi/8:
            return "뒤(180°)"
        elif yaw_normalized < 11*math.pi/8:
            return "뒤-오른쪽(225°)"
        elif yaw_normalized < 13*math.pi/8:
            return "오른쪽(270°)"
        else:
            return "앞-오른쪽(315°)"
    
    def calculate_relative_transform(self, world_to_target, world_to_source):
        """
        상대 변환 계산: source->target = inverse(world->source) * world->target
        현재는 사용되지 않지만, 향후 복잡한 좌표 변환이 필요할 때 활용 가능
        """
        result_pose = Pose()
        
        # 위치 변환: R_inv * (p_target - p_source)
        # 먼저 위치 차이 계산
        dx = world_to_target.position.x - world_to_source.position.x
        dy = world_to_target.position.y - world_to_source.position.y
        dz = world_to_target.position.z - world_to_source.position.z
        
        # 위치 차이에 역회전 적용
        source_inv = self.quaternion_inverse(world_to_source.orientation)
        
        # Quaternion 회전: p' = q * p * q_conjugate (p는 순수 quaternion으로 취급)
        # 역회전의 경우: p' = q_inv * p * q_inv_conjugate
        from geometry_msgs.msg import Quaternion
        pos_quat = Quaternion()
        pos_quat.x = dx
        pos_quat.y = dy 
        pos_quat.z = dz
        pos_quat.w = 0.0  # 순수 quaternion (실수부 = 0)
        
        # 위치 회전: source_inv * pos_quat * source_inv_conjugate
        temp = self.quaternion_multiply(source_inv, pos_quat)
        rotated_pos = self.quaternion_multiply(temp, self.quaternion_inverse(source_inv))
        
        result_pose.position.x = rotated_pos.x
        result_pose.position.y = rotated_pos.y
        result_pose.position.z = rotated_pos.z
        
        # 방향: inverse(source_orientation) * target_orientation
        result_pose.orientation = self.quaternion_multiply(source_inv, world_to_target.orientation)
        
        return result_pose


    def publish_world_to_map_transform(self):
        """원점 pose에서 정적 world->map transform 발행"""
        static_transform = TransformStamped()
        
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'world'  # 부모 프레임: Minecraft 월드 좌표계
        static_transform.child_frame_id = 'map'     # 자식 프레임: ROS map 좌표계
        
        # 첫 번째 pose 위치를 map frame의 원점으로 설정
        # 이렇게 하면 map 좌표계가 플레이어의 시작 위치를 기준으로 함
        static_transform.transform.translation.x = self.origin_pose.position.x
        static_transform.transform.translation.y = self.origin_pose.position.y
        static_transform.transform.translation.z = self.origin_pose.position.z
        
        # map 좌표계는 world와 동일한 방향으로 설정 (위치만 변환)
        # 원점의 방향을 사용하면 map 좌표계가 기울어져서 회전 시 고리 형태가 됨
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0  # identity quaternion (방향 변환 없음)
        
        # 정적 TF 브로드캐스터를 통해 발행 (한 번만 발행되고 계속 유지)
        self.static_tf_broadcaster.sendTransform(static_transform)
        self.get_logger().info(f'정적 world->map transform 발행 (위치만 변환, 방향은 identity): ({self.origin_pose.position.x:.2f}, {self.origin_pose.position.y:.2f}, {self.origin_pose.position.z:.2f})')

    def publish_map_to_odom_transform(self):
        """정적 map->odom transform 발행 (identity transform)"""
        static_transform = TransformStamped()
        
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'map'    # 부모 프레임: ROS map 좌표계
        static_transform.child_frame_id = 'odom'    # 자식 프레임: odometry 좌표계
        
        # Identity transform: map과 odom 좌표계를 동일하게 취급
        # Minecraft ground truth가 완벽하므로 추가 보정이 불필요
        static_transform.transform.translation.x = 0.0  # 위치 차이 없음
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        
        # Identity quaternion: 방향 차이 없음
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0  # 단위 quaternion
        
        # 정적 TF 브로드캐스터를 통해 발행
        self.static_tf_broadcaster.sendTransform(static_transform)
        self.get_logger().info(f'정적 map->odom identity transform이 발행되었습니다 (map과 odom 좌표계 동일)')




def main(args=None):
    """메인 함수 - ROS2 노드 실행"""
    # ROS2 초기화
    rclpy.init(args=args)
    
    # OdometryConverter 노드 생성
    odometry_converter = OdometryConverter()
    
    try:
        # 노드 실행 - 콜백 함수들이 호출되어 메시지 처리
        rclpy.spin(odometry_converter)
    except KeyboardInterrupt:
        # Ctrl+C로 종료 시 정상 종료
        pass
    finally:
        # 리소스 정리
        odometry_converter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()