#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ---------------- Package paths ----------------
    gz_pkg       = get_package_share_directory("alphabot_gazebo")
    slam_pkg     = get_package_share_directory("alphabot_slam")
    nav_pkg      = get_package_share_directory("alphabot_navigation")
    bringup_pkg  = get_package_share_directory("alphabot_bringup")

    # ---------------- Config files ----------------
    nav2_params_file = os.path.join(
        nav_pkg,
        "config",
        "nav2_params.yaml"
    )

    # --- SLAM ---
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, "launch", "online_async_slam.launch.py")
        )
    )

    # --- Nav2 (delayed by 5s to wait for odom TF from Gazebo) ---
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, "launch", "nav2_bringup.launch.py")
        ),
        launch_arguments={"params_file": nav2_params_file}.items(),
    )
    nav2_delayed = TimerAction(period=5.0, actions=[nav2_launch])

    # --- RViz ---
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "bringup_rviz.launch.py")
        )
    )

    return LaunchDescription([
        slam_launch,
        nav2_delayed,
        rviz_launch,
    ])
