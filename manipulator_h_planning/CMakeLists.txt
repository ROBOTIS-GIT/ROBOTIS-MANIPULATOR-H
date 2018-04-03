cmake_minimum_required(VERSION 2.8.3)
project(manipulator_h_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  control_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  moveit_msgs
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface
  pluginlib
  robotis_controller_msgs
  manipulator_h_demo_module_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(manipulator_h_planning
  src/manipulator_h_planning.cpp
)

add_dependencies(manipulator_h_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(manipulator_h_planning
  ${catkin_LIBRARIES}
)
