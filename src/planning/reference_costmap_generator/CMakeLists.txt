cmake_minimum_required(VERSION 3.16)
project(reference_costmap_generator)

find_package(ament_cmake REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

add_executable(${PROJECT_NAME}_node
  src/reference_costmap_generator_node.cpp
  src/reference_costmap_generator.cpp
)
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)
target_include_directories(${PROJECT_NAME}_node PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}_node
  grid_map_core
  grid_map_msgs
  grid_map_ros
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  visualization_msgs
)
target_link_libraries(${PROJECT_NAME}_node Eigen3::Eigen OpenMP::OpenMP_CXX)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
