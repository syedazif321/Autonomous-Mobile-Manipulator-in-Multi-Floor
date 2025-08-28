#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Absolute paths
    urdf_file = "/home/azif/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor/MOBILE_MANIPULATOR/mobile_manipulator_description/urdf/linear_guide.urdf.xacro"
    controller_config = "/home/azif/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor/MOBILE_MANIPULATOR/mobile_manipulator_control/config/linear_guide_controllers.yaml"
    world_file = "/home/azif/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor/ALPHA_BOT/alphabot_gazebo/worlds/no_roof_small_warehouse.world"

    # ✅ Robot description (proper xacro expansion)
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro", urdf_file]),  # <-- must be separate list items
            value_type=str
        )
    }

    # World arg
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=world_file,
        description="Gazebo world file"
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ]),
        launch_arguments={"world": LaunchConfiguration("world")}.items()
    )

    # Robot State Publisher
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen"
    )

    # ros2_control
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config, robot_description, {"use_sim_time": True}],
        output="screen"
    )

    # Spawn entity
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "linear_guide"],
        output="screen"
    )

    # Controllers
    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen"
    )
    spawner_column = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["column_position_controller", "-c", "/controller_manager"],
        output="screen"
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        robot_state_pub,
        ros2_control_node,
        spawn,
        spawner_jsb,
        spawner_column
    ])
