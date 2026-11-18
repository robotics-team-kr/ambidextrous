cmake_minimum_required(VERSION 3.22)
project(dualarm_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
# Tells MoveItConfigsBuilder where the URDF and SRDF live.
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
