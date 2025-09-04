#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    gazebo_pkg = get_package_share_directory("alphabot_gazebo")
    slam_pkg = get_package_share_directory("alphabot_slam")
    nav_pkg = get_package_share_directory("alphabot_navigation")

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, "launch", "gazebo.launch.py"))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, "launch", "spawn_alphabot.launch.py"))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(slam_pkg, "launch", "online_async_slam.launch.py"))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(nav_pkg, "launch", "nav2_bringup.launch.py"))),
    ])
