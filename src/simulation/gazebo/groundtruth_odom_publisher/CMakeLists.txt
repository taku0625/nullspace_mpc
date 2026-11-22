cmake_minimum_required(VERSION 3.16)
project(groundtruth_odom_publisher)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(${PROJECT_NAME}_node
  src/groundtruth_odom_publisher_node.cpp
  src/groundtruth_odom_publisher.cpp
)
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)
target_include_directories(${PROJECT_NAME}_node PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_ros
)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
