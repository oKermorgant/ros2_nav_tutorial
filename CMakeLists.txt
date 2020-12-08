cmake_minimum_required(VERSION 3.5)
project(ros2_nav_tutorial)

# CMake because custom service definition

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/JointMode.srv")

install(PROGRAMS src/move_joints.py DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch urdf maps params
  DESTINATION share/${PROJECT_NAME}) 

ament_package()
