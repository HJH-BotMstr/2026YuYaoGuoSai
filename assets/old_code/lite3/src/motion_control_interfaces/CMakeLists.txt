cmake_minimum_required(VERSION 3.5)
project(motion_control_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveDistance.action"
  "action/RotateAngle.action"
  "action/MoveRelative.action"
  DEPENDENCIES action_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
