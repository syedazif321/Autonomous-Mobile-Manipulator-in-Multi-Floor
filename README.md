ros2 launch custom_robots small_warehouse_with_alphabot.launch.py

ros2 launch rm_bringup rm_75_gazebo.launch.py 
ros2 launch custom_robots small_warehouse_with_alphabot.launch.py   urdf_file:=/home/azif/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor/RoboticsInfrastructure/CustomRobots/alphabot/urdf/bot.urdf   x:=0 y:=0 z:=0.1 Y:=0.0 headless:=False


ros2 launch alphabot_description view_model.launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/amazon_robot/cmd_vel

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/azif/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor/install/alphabot_description/share


export GAZEBO_PLUGIN_PATH=$HOME/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor/install/elevator_plugin/lib:$GAZEBO_PLUGIN_PATH
  

ros2 service call /elevator_cmd std_srvs/srv/SetBool "{data: true}"


ros2 launch alphabot_bringup bringup_gazebo.launch.py use_rviz:=true

ros2 topic pub /slider_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5]" -1
publisher: beginning loop
publishing #1: std_msgs.msg.Float64MultiArray(layout=std_msgs.msg.MultiArrayLayout(dim=[], data_offset=0), data=[0.5])

ros2 action send_goal /slider_position_controller/follow_joint_trajectory   control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['slider_joint'],
    points: [
      { positions: [0.0], time_from_start: {sec: 0} },
      { positions: [0.5], time_from_start: {sec: 5} }
    ]
  }
}"
ros2 topic pub /amazon_robot/cmd_vel geometry_msgs/Twist "{linear: {x: -0.3}, angular: {z: -0.2}}" -r 10
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/amazon_robot/cmd_vel


ros2 launch rm_75_config moveit_bringup.launch.py


ros2 topic pub /floor_number std_msgs/msg/Int32 "{data: 1}"
ros2 topic pub /use_floor_1 std_msgs/msg/Bool "data: true"

ros2 run alphabot_navigation map_switcher



ros2 topic pub /rm_group_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
header:
  stamp:
    sec: 0
    nanosec: 0
joint_names:
  - joint1
  - joint2
  - joint3
  - joint4
  - joint5
  - joint6
  - joint7
points:
  - positions: [-1.5105236109266285, -0.7539909602519401, -0.12728435610693634, 1.4316382470480633, -0.14625234227838213, 0.8646056681978287, 0.005513037543281918]
    time_from_start:
      sec: 3
      nanosec: 0
"

ros2 topic pub /rm_group_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
header:
  stamp:
    sec: 0
    nanosec: 0
joint_names:
  - joint1
  - joint2
  - joint3
  - joint4
  - joint5
  - joint6
  - joint7
points:
  - positions: [-1.51844, 0.40143, -0.76795, 0.64577, 0.89012, 0.66323, -0.45379]
    time_from_start:
      sec: 3
      nanosec: 0
"

