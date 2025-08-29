#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('minecraft_odometry')
    
    # Configuration file path
    cartographer_config_dir = os.path.join(pkg_dir, 'config')
    configuration_basename = 'cartographer_3d.lua'
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    cartographer_config_arg = DeclareLaunchArgument(
        'cartographer_config_dir',
        default_value=cartographer_config_dir,
        description='Full path to config file to load'
    )
    
    configuration_basename_arg = DeclareLaunchArgument(
        'configuration_basename',
        default_value=configuration_basename,
        description='Name of lua file for cartographer'
    )
    
    # Odometry configuration file argument
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'odometry_params.yaml'),
        description='Full path to parameter file'
    )
    
    # Standard SLAM: map->odom published by cartographer
    
    # Static transform removed: cartographer now tracks player frame directly
    
    # Odometry converter node
    odometry_converter_node = Node(
        package='minecraft_odometry',
        executable='odometry_converter',
        name='odometry_converter',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Sensor synchronizer node - for timestamp ordering
    sensor_synchronizer_node = Node(
        package='minecraft_odometry',
        executable='sensor_synchronizer',
        name='sensor_synchronizer',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Static transform: map -> odom (identity) - REMOVED
    # Now handled by odometry_converter for better integration
    # map_to_odom_transform = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher', 
    #     name='map_to_odom_broadcaster',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    # )
    
    # Static transform: base_link -> player (identity)
    base_link_to_player_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_player_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'player']
    )
    
    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', LaunchConfiguration('cartographer_config_dir'),
            '-configuration_basename', LaunchConfiguration('configuration_basename')
        ],
        remappings=[
            ('points2', '/points2'),
            ('imu', '/imu'),
        ]
    )
    
    # Cartographer occupancy grid node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-resolution', '0.1']
    )
    
    # PointCloud republisher and IMU republisher are assumed to be running separately
    # They are not launched here to avoid conflicts
    
    return LaunchDescription([
        use_sim_time_arg,
        cartographer_config_arg,
        configuration_basename_arg,
        config_file_arg,
        odometry_converter_node,
        sensor_synchronizer_node,
        # map_to_odom_transform,  # REMOVED: now handled by odometry_converter
        base_link_to_player_transform,
        cartographer_node,
        occupancy_grid_node
    ])