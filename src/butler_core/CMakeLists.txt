cmake_minimum_required(VERSION 3.8)
project(butler_core)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  butler_core/order_manager_node.py
  butler_core/state_machine_node.py
  butler_core/nav_bridge_node.py
  butler_core/send_order.py
  butler_core/slam_safe_velocity_limiter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
