# robomop_driver_launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Driver Node
    driver_node = Node(
        package='robomop_driver',
        executable='robomop_driver',
        name='robomop_driver_node',
        output='screen',
        parameters=[
                {'serial_port': '/dev/ROBOMOP_ARDUINO'},
                {'baudrate': 115200},
                {'wheel_radius': 0.0725},
                {'wheel_base': 0.375},
        ]
    )

    # Include Robot Description
    robot_desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('robomop_description'),
                'launch',
                'view_robomop.launch.py'
            )
        ])
    )
    
    # Include EKF Launch
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('robomop_navigation'),
                'launch',
                'sensor_ekf.launch.py'
            )
        ])
    )

    return LaunchDescription([
        driver_node,
        robot_desc_launch,
        ekf_launch
    ])

