#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_nodes(context):
    # ---- resolve args (strings) ----
    urdf_arg = LaunchConfiguration("urdf_file").perform(context)
    entity   = LaunchConfiguration("entity_name").perform(context)
    ns       = LaunchConfiguration("robot_namespace").perform(context)
    use_sim  = LaunchConfiguration("use_sim_time").perform(context)
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    Y = LaunchConfiguration("Y").perform(context)

    # ---- expand to absolute path (respect user’s relative path) ----
    urdf_path = os.path.abspath(urdf_arg)

    if not os.path.isfile(urdf_path):
        raise FileNotFoundError(f"URDF not found at: {urdf_path}")

    with open(urdf_path, "r") as f:
        robot_description_xml = f.read()

    # ---- nodes that depend on resolved values ----
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        output="screen",
        parameters=[{
            "use_sim_time": (use_sim.lower() in ("1", "true", "yes")),
            "robot_description": robot_description_xml,
        }],
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        namespace=ns,
        arguments=[
            "-entity", entity,
            "-file", urdf_path,
            "-x", x, "-y", y, "-z", z, "-Y", Y,
        ],
    )

    return [rsp, spawn]


def generate_launch_description():
    # ---- args you pass at runtime ----
    urdf_file       = DeclareLaunchArgument(
        "urdf_file",
        description="Path to the robot URDF/Xacro (absolute or relative to current directory)."
    )
    entity_name     = DeclareLaunchArgument("entity_name", default_value="mobile_robot")
    headless        = DeclareLaunchArgument("headless", default_value="False")
    use_sim_time    = DeclareLaunchArgument("use_sim_time", default_value="true")
    robot_namespace = DeclareLaunchArgument("robot_namespace", default_value="")
    x = DeclareLaunchArgument("x", default_value="0.0")
    y = DeclareLaunchArgument("y", default_value="0.0")
    z = DeclareLaunchArgument("z", default_value="0.1")
    Y = DeclareLaunchArgument("Y", default_value="0.0")

    # ---- Gazebo env (keep simple) ----
    set_plugin_path = SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=os.pathsep.join(filter(None, [
            "/opt/ros/humble/lib",
            os.environ.get("GAZEBO_PLUGIN_PATH", ""),
        ]))
    )

    # ---- start Gazebo Classic (empty world) ----
    start_gzserver = ExecuteProcess(
        cmd=["gzserver", "--verbose", "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )
    start_gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )

    # ---- build nodes after args are resolved ----
    build = OpaqueFunction(function=_build_nodes)

    ld = LaunchDescription()
    ld.add_action(urdf_file)
    ld.add_action(entity_name)
    ld.add_action(headless)
    ld.add_action(use_sim_time)
    ld.add_action(robot_namespace)
    ld.add_action(x)
    ld.add_action(y)
    ld.add_action(z)
    ld.add_action(Y)

    ld.add_action(set_plugin_path)
    ld.add_action(start_gzserver)
    ld.add_action(start_gzclient)
    ld.add_action(build)
    return ld
