#!/usr/bin/env python3
"""
Manual Mode Launch File (final)

Provides:
 - Mapping (slam_toolbox)
 - Manual control stack
 - Map saving service for ModeManager
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    blitz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('blitz'),
                'launch',
                'blitz.launch.py'
            )
        )
    )

    odom_node = Node(
        package='rover',
        executable='odom_node',
        name='odom_node',
        output='screen',
        parameters=[
            {'wheel_diameter': 0.11},
            {'wheel_base': 0.35},
            {'ticks_per_rev': 3800},
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
            {'publish_tf': True},
            {'publish_rate': 50.0},
        ]
    )

    velocity_bridge = Node(
        package='rover',
        executable='velocity_bridge',
        name='velocity_bridge',
        output='screen'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            
            # Critical: Increase transform tolerance
            'transform_timeout': 0.5,
            'tf_buffer_duration': 30.0,
            
            # Smoother odometry handling
            'minimum_travel_distance': 0.2,
            'minimum_travel_heading': 0.2,
            
            # Better scan matching
            'scan_buffer_size': 10,
            'scan_buffer_maximum_scan_distance': 20.0,
            'link_match_minimum_response_fine': 0.1,
            'link_scan_maximum_distance': 1.5,
            
            # Loop closure
            'loop_search_maximum_distance': 3.0,
            'do_loop_closing': True,
            'loop_match_minimum_chain_size': 10,
            'loop_match_maximum_variance_coarse': 3.0,
            
            # Performance
            'use_scan_matching': True,
            'use_scan_barycenter': True,
            'correlation_search_space_dimension': 0.5,
            'correlation_search_space_resolution': 0.01,
            'correlation_search_space_smear_deviation': 0.1,
        }]
    )

    auto_map_saver = Node(
        package='rover',
        executable='auto_map_saver',
        name='auto_map_saver',
        output='screen',
        parameters=[
            {'save_directory': os.path.join(os.path.expanduser('~'), 'maps')}
        ]
    )

    mode_manager = Node(
        package='rover',
        executable='mode_manager',
        name='mode_manager',
        output='screen'
    )

    # Fixed: Correct frame name and argument order
    static_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '--x', '0.18',
            '--y', '0.0',
            '--z', '0.0',
            '--qx', '0.0',
            '--qy', '0.0',
            '--qz', '0.0',
            '--qw', '1.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser_frame'
        ]
    )

    static_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.02',
            '--qx', '0.0',
            '--qy', '0.0',
            '--qz', '0.0',
            '--qw', '1.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu'
        ]
    )

    return LaunchDescription([
        odom_node,
        static_base_to_laser,
        static_base_to_imu,
        velocity_bridge,
        slam_toolbox_node,
        auto_map_saver,
        mode_manager,
    ])