#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to params
    params = os.path.join(
        get_package_share_directory("alphabot_navigation"),
        "config",
        "nav2_params.yaml"
    )

    return LaunchDescription([
        # Controller server
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params],
        ),

        # Planner server
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params],
        ),

        # Behavior Tree navigator
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params],
        ),

        # Behavior server (needed for spin, backup, etc.)
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params],
        ),

        # Lifecycle manager
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "bt_navigator",
                    "behavior_server",   # added here
                ],
            }],
        ),
    ])
