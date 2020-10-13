cmake_minimum_required(VERSION 3.5)
project(ros2_nav_tutorial)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/ChangeMode.srv")

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

# Laser simulation node
add_library(simulation_node SHARED
  include/ros2_nav_tutorial/occupancy_grid.h src/occupancy_grid.cpp
  include/ros2_nav_tutorial/laser_scanner.h src/laser_scanner.cpp
  include/ros2_nav_tutorial/simulation.h src/simulation.cpp)
target_include_directories(simulation_node PRIVATE ${OpenCV_INCLUDE_DIRS} ${YamlCpp_INCLUDE_DIRS})
ament_target_dependencies(simulation_node
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs)
target_link_libraries(simulation_node ${OpenCV_LIBRARIES} yaml-cpp stdc++fs)

## Robot control node
add_library(robot_node SHARED
  include/ros2_nav_tutorial/robot.h src/robot.cpp)
ament_target_dependencies(robot_node
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  tf2_ros
  std_srvs)
rosidl_target_interfaces(robot_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()


# auto-executable from node
rclcpp_components_register_node(simulation_node
  PLUGIN "ros2_nav_tutorial::Simulation"
  EXECUTABLE simulation)

# auto-executable from node
rclcpp_components_register_node(robot_node
  PLUGIN "ros2_nav_tutorial::Robot"
  EXECUTABLE robot)

install(TARGETS simulation_node robot_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  
install(DIRECTORY launch urdf maps params
  DESTINATION share/${PROJECT_NAME}) 

ament_export_libraries(simulation_node)
ament_package()
