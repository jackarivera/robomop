# robomop_simulator/launch/bridge.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Bridge for /cmd_vel
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
            output='screen'
        ),

        # Bridge for /odometry/wheels
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='odometry_bridge',
            arguments=['/odometry/wheels@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
            output='screen'
        ),

        # Bridge for /tf_gazebo
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='tf_bridge',
            arguments=['/tf_gazebo@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'],
            output='screen'
        ),

        # Bridge for /joint_states
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='joint_state_bridge',
            arguments=['/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model'],
            output='screen'
        ),

        # Bridge for /scan (Lidar)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='scan_bridge',
            arguments=['/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
            output='screen'
        ),

        # Bridge for /imu/data (If applicable)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='imu_bridge',
            arguments=['/imu/data@sensor_msgs/msg/Imu@ignition.msgs.IMU'],
            output='screen'
        ),
    ])
