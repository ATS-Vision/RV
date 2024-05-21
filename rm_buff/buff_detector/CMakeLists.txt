cmake_minimum_required(VERSION 3.10)
project(buff_detector)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(buff_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include 
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OPENVINO_INCLUDE_DIRS}
)

set(dependencies 
  rclcpp 
  rclcpp_components
  OpenCV
  OpenVINO
  geometry_msgs
  Eigen3 
  rclpy 
  message_filters
  image_transport
  cv_bridge
  buff_interfaces
  visualization_msgs
  tf2_geometry_msgs
  tf2_ros
)


add_library(${PROJECT_NAME} SHARED
src/window.cpp
src/pnp_solver.cpp
src/detector.cpp 
src/detector_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)



target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS}  ${OPENVINO_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS}  openvino::runtime)

add_executable(${PROJECT_NAME}_node 
  src/window.cpp
  src/pnp_solver.cpp
  src/detector.cpp 
  src/detector_node.cpp
  )

ament_target_dependencies(${PROJECT_NAME}_node 
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_node 
  openvino::runtime 
  
)

rclcpp_components_register_nodes(${PROJECT_NAME} 
  PLUGIN rm_buff::BuffDetectorNode
  EXECUTABLE buff_detector_node  
)

install(TARGETS 
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(TARGETS 
  ${PROJECT_NAME}_node 
  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  ament_add_gtest(test_node_startup test/test_node_startup.cpp)
  target_link_libraries(test_node_startup ${PROJECT_NAME})

  ament_add_gtest(test_number_cls test/test_number_cls.cpp)
  target_link_libraries(test_number_cls ${PROJECT_NAME})

endif()


# ament_auto_package(
#   INSTALL_TO_SHARE
#   model
# )

ament_package()
