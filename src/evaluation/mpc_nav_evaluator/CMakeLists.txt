cmake_minimum_required(VERSION 3.16)
project(mpc_nav_evaluator)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/mpc_nav_evaluator.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
