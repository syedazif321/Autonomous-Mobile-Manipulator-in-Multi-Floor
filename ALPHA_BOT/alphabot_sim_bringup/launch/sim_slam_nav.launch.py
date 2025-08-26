from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    gz_pkg = get_package_share_directory("alphabot_gazebo")
    ctrl_pkg = get_package_share_directory("alphabot_control")
    slam_pkg = get_package_share_directory("alphabot_slam")
    nav_pkg  = get_package_share_directory("alphabot_navigation")
    bringup_pkg = get_package_share_directory("alphabot_bringup")

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gz_pkg, "launch", "spawn_alphabot.launch.py"))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(ctrl_pkg, "launch", "control.launch.py"))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(slam_pkg, "launch", "online_async_slam.launch.py"))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(nav_pkg,  "launch", "nav2_bringup.launch.py"))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup_pkg,"launch","bringup_rviz.launch.py"))),
    ])
