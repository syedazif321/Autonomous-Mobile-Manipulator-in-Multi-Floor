from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params = os.path.join(get_package_share_directory("alphabot_navigation"), "config", "nav2_params.yaml")
    return LaunchDescription([
        Node(package="nav2_controller", executable="controller_server",
             parameters=[params, {"use_sim_time": True}], output="screen"),
        Node(package="nav2_planner", executable="planner_server",
             parameters=[params, {"use_sim_time": True}], output="screen"),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             parameters=[params, {"use_sim_time": True}], output="screen"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation",
             parameters=[{"use_sim_time": True},
                         {"autostart": True},
                         {"node_names": ["controller_server","planner_server","bt_navigator"]}],
             output="screen"),
    ])
