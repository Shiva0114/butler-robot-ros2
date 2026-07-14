cmake_minimum_required(VERSION 3.8)
project(butler_monitor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  butler_monitor/robot_monitor_node.py
  butler_monitor/timeout_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
