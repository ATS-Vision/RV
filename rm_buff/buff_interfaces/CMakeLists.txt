cmake_minimum_required(VERSION 3.8)
project(buff_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Buff.msg"
  "msg/Point2f.msg"
  "msg/Buffs.msg"
  "msg/Target.msg"
  "msg/TargetInfo.msg"
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
  sensor_msgs
  OpenCV
)

ament_package()