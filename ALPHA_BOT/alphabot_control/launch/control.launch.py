from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("alphabot_control"), "config", "controllers.yaml")
    cm = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[cfg, {"use_sim_time": True}],
        output="screen",
    )
    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    diff = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base", "-c", "/controller_manager"],
        output="screen",
    )
    return LaunchDescription([cm, jsb, diff])
