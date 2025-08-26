ros2 launch custom_robots small_warehouse_with_ackermann_logistic_robot.launch.py

ros2 launch rm_bringup rm_75_gazebo.launch.py 
ros2 launch custom_robots small_warehouse_with_ackermann_logistic_robot.launch.py   urdf_file:=/home/azif/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor/RoboticsInfrastructure/CustomRobots/ackermann_logistic_robot/urdf/bot.urdf   x:=0 y:=0 z:=0.1 Y:=0.0 headless:=False
