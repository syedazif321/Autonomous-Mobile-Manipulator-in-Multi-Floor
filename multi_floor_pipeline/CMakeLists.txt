cmake_minimum_required(VERSION 3.8)
project(multi_floor_pipeline)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(msg_gazebo REQUIRED)

include_directories(include)

# Main executable
add_executable(pipeline_main src/pipeline_main.cpp)
ament_target_dependencies(pipeline_main
  rclcpp
  behaviortree_cpp_v3
  geometry_msgs  
  nav2_msgs  
  control_msgs    
  msg_gazebo
  std_srvs
)

# Install executable
install(TARGETS pipeline_main
  DESTINATION lib/${PROJECT_NAME}
)

# Install other resources
install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
