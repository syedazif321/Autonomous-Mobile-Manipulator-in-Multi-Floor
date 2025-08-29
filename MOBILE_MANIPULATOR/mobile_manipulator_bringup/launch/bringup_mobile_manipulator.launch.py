#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # ---------------- Packages ----------------
    pkg_desc = get_package_share_directory("mobile_manipulator_description")
    pkg_gazebo = get_package_share_directory("alphabot_gazebo")  # reuse alphabot worlds
    # pkg_moveit = get_package_share_directory("rm_75_config")

    # ---------------- Arguments ----------------
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([pkg_gazebo, "worlds", "no_roof_small_warehouse.world"]),
        description="SDF world file"
    )
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    entity_arg = DeclareLaunchArgument("entity", default_value="mobile_manipulator")

    # ---------------- Robot Description (URDF/Xacro) ----------------
    urdf_file = os.path.join(pkg_desc, "urdf", "mobile_manipulator.urdf.xacro")
    robot_description_content = Command([FindExecutable(name="xacro"), " ", urdf_file])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=None)}

    # ---------------- Gazebo ----------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, "launch", "gazebo.launch.py")),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    # ---------------- Robot State Publisher ----------------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[("joint_states", "/joint_states")],
        output="screen"
    )

    # ---------------- Spawn Entity in Gazebo ----------------
    spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", LaunchConfiguration("entity"), "-topic", "robot_description"],
        output="screen",
    )

    # ---------------- Controller Spawners ----------------
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen"
    )

    base_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base", "-c", "/controller_manager"],
        output="screen"
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        output="screen"
    )

    # ---------------- MoveIt 2 (for the Arm) ----------------
    # moveit_config = MoveItConfigsBuilder(
    #     "rm_moveit2_config",  # matches SRDF/URDF naming
    #     package_name="rm_75_config"
    # ).to_moveit_configs()

    # move_group = Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     output="screen",
    #     parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    # )

    # rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
    #     parameters=[moveit_config.to_dict(),
    #                 moveit_config.robot_description_kinematics,
    #                 {"use_sim_time": True}],
    #     output="screen"
    # )


    slider_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["slider_position_controller", "-c", "/controller_manager"],
        output="screen"
    )

    return LaunchDescription([
        world_arg, use_sim_time_arg, entity_arg,
        gazebo,
        rsp,
        spawner,
        jsb_spawner,
        base_spawner,
        arm_spawner,
        slider_spawner,
        # move_group,
        # rviz,
    ])
