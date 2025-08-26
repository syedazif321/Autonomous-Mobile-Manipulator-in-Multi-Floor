from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    world_file = os.path.join(get_package_share_directory("alphabot_gazebo"), "worlds", "empty.world")
    world = DeclareLaunchArgument("world", default_value=world_file)
    use_sim_time = SetParameter(name="use_sim_time", value=True)
    gz = ExecuteProcess(cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so", LaunchConfiguration("world")],
                        output="screen")
    return LaunchDescription([world, use_sim_time, gz])
