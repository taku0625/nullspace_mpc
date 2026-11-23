cmake_minimum_required(VERSION 3.16)
project(mppi_unicycle)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(misachiya_drive REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowPath.action"
  DEPENDENCIES nav_msgs
)

add_library(mppi_unicycle_core STATIC src/core.cpp)
target_compile_features(mppi_unicycle_core PUBLIC cxx_std_17)
target_include_directories(mppi_unicycle_core PUBLIC include)
target_link_libraries(mppi_unicycle_core PRIVATE OpenMP::OpenMP_CXX)

add_executable(follow_path_node src/follow_path_node.cpp)
ament_target_dependencies(follow_path_node geometry_msgs grid_map_core grid_map_msgs grid_map_ros
  misachiya_drive nav_msgs rclcpp rclcpp_action std_msgs tf2 tf2_geometry_msgs tf2_ros
  visualization_msgs)
rosidl_get_typesupport_target(action_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(follow_path_node mppi_unicycle_core "${action_typesupport}")

install(TARGETS follow_path_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_core test/test_core.cpp)
  target_link_libraries(test_core mppi_unicycle_core)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
