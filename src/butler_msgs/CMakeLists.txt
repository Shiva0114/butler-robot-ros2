cmake_minimum_required(VERSION 3.8)
project(butler_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "srv/PlaceOrder.srv"
  "srv/CancelOrder.srv"
  "action/DeliverOrder.action"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
