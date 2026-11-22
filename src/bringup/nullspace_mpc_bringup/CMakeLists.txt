cmake_minimum_required(VERSION 3.16)
project(nullspace_mpc_bringup)

find_package(ament_cmake REQUIRED)

set(repo_root "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
install(DIRECTORY "${repo_root}/launch/" DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY "${repo_root}/data/map" "${repo_root}/data/rviz" DESTINATION share/${PROJECT_NAME})
install(FILES "${repo_root}/config/nav2.yaml" "${repo_root}/config/slam_toolbox.yaml"
  DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS scripts/goal_pose_bridge scripts/run_evaluation.py DESTINATION lib/${PROJECT_NAME})

ament_package()
