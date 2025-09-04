#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    pkg_desc = get_package_share_directory("mobile_manipulator_description")
    pkg_gazebo = get_package_share_directory("alphabot_gazebo")

    # --- Args ---
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([pkg_gazebo, "worlds", "no_roof_small_warehouse.world"]),
        description="SDF world file"
    )
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    entity_arg = DeclareLaunchArgument("entity", default_value="mobile_manipulator")

    # --- Robot Description ---
    urdf_file = os.path.join(pkg_desc, "urdf", "mobile_manipulator.urdf.xacro")
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", urdf_file]), value_type=None
        )
    }

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, "launch", "gazebo.launch.py")),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    # --- State publisher ---
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen"
    )

    # --- Spawn robot entity ---
    spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", LaunchConfiguration("entity"), "-topic", "robot_description"],
        output="screen"
    )

    # --- Controllers ---
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen"
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rm_group_controller", "-c", "/controller_manager"],
        output="screen"
    )

    slider_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["slider_position_controller", "-c", "/controller_manager"],
        output="screen"
    )

    # --- Event chaining ---
    load_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawner,
            on_exit=[jsb_spawner],
        )
    )

    load_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[arm_spawner],
        )
    )

    load_slider = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_spawner,
            on_exit=[slider_spawner],
        )
    )

    return LaunchDescription([
        world_arg, use_sim_time_arg, entity_arg,
        gazebo,
        rsp,
        spawner,
        load_jsb,
        load_arm,
        load_slider,
    ])
