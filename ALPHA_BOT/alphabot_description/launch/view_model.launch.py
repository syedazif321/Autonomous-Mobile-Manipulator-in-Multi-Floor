from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # --- Args ---
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([
            get_package_share_directory("alphabot_gazebo"),
            "worlds", "empty.world"
        ]),
        description="SDF world file"
    )
    entity_arg = DeclareLaunchArgument("entity", default_value="alphabot")
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_arg = DeclareLaunchArgument("z", default_value="0.1")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")

    world = LaunchConfiguration("world")
    entity = LaunchConfiguration("entity")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")

    # --- Robot description from Xacro (RELATIVE paths only) ---
    urdf_file = PathJoinSubstitution([
        get_package_share_directory("alphabot_description"),
        "urdf", "alphabot.urdf.xacro",
    ])
    # IMPORTANT: Command concatenates tokens; add a literal ' ' between them
    robot_description_cmd = Command([FindExecutable(name="xacro"), " ", urdf_file])
    robot_description = ParameterValue(robot_description_cmd, value_type=str)

    # --- Gazebo (Classic) server+client ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("alphabot_gazebo"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    # --- Robot State Publisher (so TF/robot_description exist for spawn_entity) ---
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }],
        output="screen",
    )

    # --- Spawn entity in Gazebo from /robot_description ---
    spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", entity,
            "-topic", "robot_description",
            "-x", x, "-y", y, "-z", z, "-Y", yaw
        ],
        output="screen",
    )

    return LaunchDescription([
        world_arg, entity_arg, x_arg, y_arg, z_arg, yaw_arg, use_sim_time_arg,
        gazebo, rsp, spawner
    ])
