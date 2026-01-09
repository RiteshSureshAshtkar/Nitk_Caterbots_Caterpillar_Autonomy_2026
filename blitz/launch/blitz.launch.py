from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for ESP32 communication'
    )
    
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='Serial baud rate'
    )
    
    # Unified Blitz node (handles both read and write on single serial connection)
    blitz_node = Node(
        package='blitz',
        executable='blitz_node.py',
        name='blitz_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'reconnect_interval': 2.0
        }]
    )
    
    return LaunchDescription([
        port_arg,
        baud_arg,
        blitz_node
    ])
