cmake_minimum_required(VERSION 3.16)
project(map_visualizer)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(${PROJECT_NAME}_node
  src/map_visualizer_node.cpp
  src/map_visualizer.cpp
)
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)
target_include_directories(${PROJECT_NAME}_node PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}_node
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
