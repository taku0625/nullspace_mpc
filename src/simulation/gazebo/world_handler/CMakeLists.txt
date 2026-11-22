cmake_minimum_required(VERSION 3.16)
project(world_handler)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz world xacro models DESTINATION share/${PROJECT_NAME})

ament_package()
