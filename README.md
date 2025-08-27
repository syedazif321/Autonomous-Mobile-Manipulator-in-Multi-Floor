ros2 launch custom_robots small_warehouse_with_alphabot.launch.py

ros2 launch rm_bringup rm_75_gazebo.launch.py 
ros2 launch custom_robots small_warehouse_with_alphabot.launch.py   urdf_file:=/home/azif/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor/RoboticsInfrastructure/CustomRobots/alphabot/urdf/bot.urdf   x:=0 y:=0 z:=0.1 Y:=0.0 headless:=False


ros2 launch alphabot_description view_model.launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/amazon_robot/cmd_vel

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/azif/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor/install/alphabot_description/share

ros2 run elevator_plugin elev_cmd 1