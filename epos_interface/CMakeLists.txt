cmake_minimum_required(VERSION 3.5)
project(epos_interface)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ClearFault.srv"
  "srv/SetEnable.srv"
  "srv/SetOperationMode.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
