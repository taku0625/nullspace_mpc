cmake_minimum_required(VERSION 3.16)
project(joy_controller)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(${PROJECT_NAME}_node
  src/joy_controller_node.cpp
  src/joy_controller.cpp
)
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)
target_include_directories(${PROJECT_NAME}_node PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}_node
  geometry_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
