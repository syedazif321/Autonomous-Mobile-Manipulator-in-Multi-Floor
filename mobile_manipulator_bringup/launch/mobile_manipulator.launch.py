from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf = "/home/azif/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor/mobile_manipulator_bringup/urdf/linear_guide.urdf"

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    os.getenv('COLCON_PREFIX_PATH').split(':')[0],
                    'share', 'gazebo_ros', 'launch', 'gazebo.launch.py'
                )
            )
        ),

        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-entity", "linear_guide", "-file", urdf],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[urdf, "/home/azif/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor/mobile_manipulator_bringup/config/linear_guide_controllers.yaml"],
            output="screen"
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["column_position_controller"],
        ),
    ])
