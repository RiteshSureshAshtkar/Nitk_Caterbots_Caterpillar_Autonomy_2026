#!/usr/bin/env python3
"""
Autonomous Mode Launch File (Nav2 + Crater Avoidance)

Called by Mode Manager when switching to autonomous mode.

Architecture:
  Nav2 -> /cmd_vel -> [Velocity Bridge] -> /velocity -> MCU/Blitz
                           ^
                           |
                      /ml_pipeline (crater detection)
                      
Behavior:
 - DOES NOT launch slam_toolbox (map-based navigation only)
 - Prompts user to select a saved map at launch
 - Launches Nav2 for autonomous navigation
 - Launches Velocity Bridge for crater avoidance
 - Mode is already set to AUTONOMOUS by ModeSwitch publisher
 - When crater detected: Bridge overrides Nav2 commands temporarily
 - After avoidance: Nav2 resumes control and continues to goal
"""
import os
from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

MAPS_DIR = os.path.join(os.path.expanduser('~'), 'maps')

def launch_autonomous_system(context, *args, **kwargs):
    """Launch Nav2 + Velocity Bridge with selected map."""
    
    # Ask user for map index
    map_index = input(
        '\n' + '='*60 + '\n'
        'AUTONOMOUS MODE LAUNCHER\n'
        '='*60 + '\n'
        'Enter map number to load (e.g., 1, 2, 3): '
    ).strip()
    
    if not map_index.isdigit():
        raise RuntimeError(
            f'Invalid input "{map_index}". Expected a numeric map index.'
        )
    
    yaml_path = os.path.join(MAPS_DIR, f'{map_index}.yaml')
    pgm_path = os.path.join(MAPS_DIR, f'{map_index}.pgm')
    
    if not os.path.exists(yaml_path) or not os.path.exists(pgm_path):
        available_maps = []
        if os.path.exists(MAPS_DIR):
            available_maps = [f.replace(".yaml", "") for f in os.listdir(MAPS_DIR) if f.endswith(".yaml")]
        
        raise RuntimeError(
            f'\nMap files not found:\n'
            f'  YAML: {yaml_path}\n'
            f'  PGM:  {pgm_path}\n\n'
            f'Maps directory: {MAPS_DIR}\n'
            f'Available maps: {", ".join(available_maps) if available_maps else "None"}'
        )
    
    print(f'\n{"="*60}')
    print(f'Selected Map: {map_index}.yaml')
    print(f'Path: {yaml_path}')
    print(f'{"="*60}\n')
    
    # ========================================
    # 1. Nav2 Bringup (with selected map)
    # ========================================
    nav2_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )
    
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'map': yaml_path,
            'use_sim_time': 'False',
            'autostart': 'True'
        }.items()
    )
    
    # ========================================
    # 2. Velocity Bridge (Crater Avoidance)
    # ========================================
    # This node intercepts /cmd_vel from Nav2 and adds crater avoidance
    velocity_bridge = Node(
        package='rover',  # Your package name
        executable='velocity_bridge',  # The enhanced velocity bridge node
        name='velocity_bridge',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/velocity',
            'max_linear_vel': 1.0,
            'max_angular_vel': 2.0,
            'avoidance_linear_vel': 0.3,
            'avoidance_angular_vel': 0.5,
            'backward_distance': 0.2
        }]
    )
    
    # ========================================
    # 3. RViz2 (Visualization)
    # ========================================
    rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # ========================================
    # 4. (Optional) Test ML Pipeline Publisher
    # ========================================
    # Uncomment to test crater avoidance without real ML pipeline
    # test_ml_publisher = Node(
    #     package='rover',
    #     executable='test_crater_publisher',
    #     name='test_crater_publisher',
    #     output='screen'
    # )
    
    return [
        LogInfo(msg='\n' + '='*60),
        LogInfo(msg='AUTONOMOUS NAVIGATION MODE'),
        LogInfo(msg='='*60),
        LogInfo(msg=f'Map:          {map_index}.yaml'),
        LogInfo(msg='SLAM:         DISABLED (using saved map)'),
        LogInfo(msg='Nav2:         ENABLED'),
        LogInfo(msg='Crater Avoid: ENABLED'),
        LogInfo(msg='Mode:         AUTONOMOUS (set by ModeSwitch)'),
        LogInfo(msg='='*60 + '\n'),
        
        nav2,
        velocity_bridge,
        rviz,
        # test_ml_publisher,  # Uncomment for testing
    ]

def generate_launch_description():
    """Generate launch description."""
    
    ld = LaunchDescription()
    
    # Welcome message
    ld.add_action(
        LogInfo(
            msg='\nAutonomous Mode: Map-based navigation with crater avoidance\n'
                'Launched by Mode Manager'
        )
    )
    
    # Interactive map selection and launch
    ld.add_action(
        OpaqueFunction(function=launch_autonomous_system)
    )
    
    return ld