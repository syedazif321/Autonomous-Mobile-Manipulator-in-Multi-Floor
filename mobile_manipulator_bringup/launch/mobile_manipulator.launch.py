#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration(
        'world',
        default=os.path.join(get_package_share_directory('bcr_bot'), 'worlds', 'small_warehouse.sdf')
    )

    # ONE robot_description for the combined model
    combined_xacro = PathJoinSubstitution([FindPackageShare('mobile_manipulator_bringup'), 'urdf', 'mobile_manipulator.xacro'])
    robot_description = Command(['xacro ', combined_xacro])

    # Gazebo (server+client)
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world}.items()
    )

    # Single RSP publishing the combined tree
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': robot_description}],
        output='screen'
    )

    # Spawn combined entity from robot_description
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'bcr_combo'],
        output='screen'
    )

    # Minimal controllers we add here (JSB). Base and arm controllers are loaded by their own YAMLs/plugins.
    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster'], output='screen')

    # Your existing Nav2 bringup (kept exactly as you have it)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bcr_bot'), 'launch', 'nav2.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=world),
        gz, rsp, spawn, jsb,
        nav2_launch
    ])
