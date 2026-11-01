cmake_minimum_required(VERSION 3.8)
project(linebot_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/generate_track.py DESTINATION lib/${PROJECT_NAME})

ament_package()
