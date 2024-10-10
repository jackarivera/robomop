from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robomop_driver',
            executable='robomop_driver',
            name='robomop_driver_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyACM0'},
                {'baudrate': 115200},
                {'wheel_radius': 0.0725},
                {'wheel_base': 0.375},
            ]
        )
    ])

