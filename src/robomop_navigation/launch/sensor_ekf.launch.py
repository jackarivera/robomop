# ekf_launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the EKF config file
    ekf_config = os.path.join(
        get_package_share_directory('robomop_navigation'),
        'config',
        'robomop_ekf.yaml'
    )

    # EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    return LaunchDescription([
        ekf_node
    ])
