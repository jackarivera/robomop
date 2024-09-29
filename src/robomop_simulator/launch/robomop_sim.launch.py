# robomop_simulator/launch/robomop_sim.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  # Import ParameterValue


def generate_launch_description():
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    # Paths to various files and directories
    robomop_description_share = get_package_share_directory('robomop_description')
    robomop_simulator_share = get_package_share_directory('robomop_simulator')
    robomop_driver_share = get_package_share_directory('robomop_driver')

    urdf_xacro = os.path.join(robomop_description_share, 'urdf', 'robomop.urdf.xacro')
    controller_config = os.path.join(robomop_driver_share, 'config', 'controller.yaml')
    rviz_config = os.path.join(robomop_simulator_share, 'config', 'robomop.rviz')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='maze.sdf',
        description='World file to use in Gazebo'
    )

    # Path to the world file within robomop_simulator package
    world_path = PathJoinSubstitution([
        robomop_simulator_share, 'worlds', world
    ])

    # Include the ros_gz_sim launch file
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    ros_gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": world_path
        }.items()
    )

    # Spawn the robot in Gazebo using ros_gz_sim's create node
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "robomop",
            "-allow_renaming", "true",
            "-x", "0",  # Initial x position
            "-y", "0",  # Initial y position
            "-z", "0.1"  # Initial z position
        ],
        output="screen"
    )

    # ros2_control node with XACRO processing
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_xacro]),
        value_type=str  # Ensure it's treated as a string
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }, controller_config],
        output='screen'
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Diff Drive Controller
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )

    # Launch ros_gz_bridge
    bridge_launch = os.path.join(
        robomop_simulator_share, 'launch', 'bridge.launch.py'
    )
    ros_gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            bridge_launch
        )
    )

    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }],
        arguments=[]
    )

    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)

    # Launch ros_gz_sim
    ld.add_action(ros_gz_sim_launch)

    # Spawn the robot
    ld.add_action(gz_spawn_entity)

    # Launch ros2_control_node
    ld.add_action(ros2_control_node)

    # Spawn controllers
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(diff_drive_controller_spawner)

    # Launch ros_gz_bridge
    ld.add_action(ros_gz_bridge)

    # Launch Robot State Publisher
    ld.add_action(robot_state_publisher_cmd)

    # Launch RViz
    ld.add_action(rviz_node)

    return ld
