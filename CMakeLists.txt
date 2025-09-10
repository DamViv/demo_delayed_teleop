cmake_minimum_required(VERSION 3.5)
project(demo_delayed_teleop)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclpy REQUIRED)

# Node C++
add_executable(delay_node src/delay_node.cpp)
ament_target_dependencies(delay_node rclcpp geometry_msgs sensor_msgs message_filters)
install(TARGETS delay_node DESTINATION lib/${PROJECT_NAME})

# Installer le package Python
ament_python_install_package(${PROJECT_NAME})

# Installer les scripts Python exécutables
install(PROGRAMS
  ${PROJECT_NAME}/safety_node.py
  ${PROJECT_NAME}/color_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# Installer le dossier launch
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
