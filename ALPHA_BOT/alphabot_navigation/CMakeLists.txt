cmake_minimum_required(VERSION 3.8)
project(alphabot_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install launch files, URDFs, meshes, configs, worlds, maps
install(DIRECTORY launch config urdf meshes worlds maps
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install Python nodes as executables
install(PROGRAMS
  scripts/scan_filter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
