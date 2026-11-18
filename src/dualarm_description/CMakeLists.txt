cmake_minimum_required(VERSION 3.22)
project(dualarm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config worlds launch DESTINATION share/${PROJECT_NAME})

ament_package()
