from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    gz = os.path.join(get_package_share_directory("alphabot_gazebo"), "launch", "gazebo.launch.py")
    desc = os.path.join(get_package_share_directory("alphabot_description"), "urdf", "alphabot.urdf.xacro")

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": f"$(xacro {desc})"}, {"use_sim_time": True}],
        output="screen",
    )
    spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "alphabot", "-topic", "robot_description"],
        output="screen",
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz)),
        rsp,
        spawner
    ])
