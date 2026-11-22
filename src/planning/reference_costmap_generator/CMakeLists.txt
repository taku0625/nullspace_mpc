cmake_minimum_required(VERSION 3.16)
project(reference_costmap_generator)

find_package(ament_cmake REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
add_library(reference_costmap STATIC src/reference_costmap_generator.cpp)
target_compile_features(reference_costmap PUBLIC cxx_std_17)
target_include_directories(reference_costmap PUBLIC include)
ament_target_dependencies(reference_costmap grid_map_core nav_msgs rclcpp)

add_executable(${PROJECT_NAME}_node src/reference_costmap_generator_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node grid_map_msgs grid_map_ros)
target_link_libraries(${PROJECT_NAME}_node reference_costmap)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_reference_costmap test/test_reference_costmap.cpp)
  target_link_libraries(test_reference_costmap reference_costmap)
endif()

ament_package()
