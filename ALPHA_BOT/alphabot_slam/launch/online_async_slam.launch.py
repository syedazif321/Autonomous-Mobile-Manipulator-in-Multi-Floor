#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    slam_pkg = get_package_share_directory("slam_toolbox")
    alphabot_slam = get_package_share_directory("alphabot_slam")

    slam_params = os.path.join(alphabot_slam, "config", "slam_toolbox.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_pkg, "launch", "online_async_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params
        }.items()
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(get_package_share_directory("alphabot_bringup"), "config", "alphabot.rviz")]
    )

    return LaunchDescription([slam_toolbox, rviz])
