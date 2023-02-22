cmake_minimum_required(VERSION 3.5)
project(yolo_detector_node)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yolo_interface REQUIRED)
ament_auto_find_build_dependencies(REQUIRED)

add_executable(publisher src/yolo_detector_node.cpp)
ament_target_dependencies(publisher rclcpp yolo_interface)

install(TARGETS
  publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
