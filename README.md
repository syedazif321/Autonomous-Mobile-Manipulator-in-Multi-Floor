ros2 launch custom_robots small_warehouse_with_alphabot.launch.py

ros2 launch rm_bringup rm_75_gazebo.launch.py 
ros2 launch custom_robots small_warehouse_with_alphabot.launch.py   urdf_file:=/home/azif/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor/RoboticsInfrastructure/CustomRobots/alphabot/urdf/bot.urdf   x:=0 y:=0 z:=0.1 Y:=0.0 headless:=False


ros2 launch alphabot_description view_model.launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/amazon_robot/cmd_vel

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/azif/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor/install/alphabot_description/share


export GAZEBO_PLUGIN_PATH=$HOME/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor/install/elevator_plugin/lib:$GAZEBO_PLUGIN_PATH
  

ros2 service call /elevator_cmd std_srvs/srv/SetBool "{data: true}"


ros2 launch alphabot_bringup bringup_gazebo.launch.py use_rviz:=true

azif@azif:~/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor$ ros2 topic pub /slider_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5]" -1
publisher: beginning loop
publishing #1: std_msgs.msg.Float64MultiArray(layout=std_msgs.msg.MultiArrayLayout(dim=[], data_offset=0), data=[0.5])

azif@azif:~/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor$ ros2 action send_goal /slider_position_controller/follow_joint_trajectory   control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['slider_joint'],
    points: [
      { positions: [0.0], time_from_start: {sec: 0} },
      { positions: [0.5], time_from_start: {sec: 5} }
    ]
  }
}"
