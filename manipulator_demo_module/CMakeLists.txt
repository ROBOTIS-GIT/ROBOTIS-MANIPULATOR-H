cmake_minimum_required(VERSION 2.8.3)
project(manipulator_demo_module)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  robotis_device
  robotis_math
  robotis_controller_msgs
  robotis_framework_common
  roscpp
  std_msgs
  geometry_msgs
  moveit_msgs
  moveit_core
  manipulator_kinematics_dynamics
  manipulator_demo_module_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  cmake_modules
)

find_package(Eigen REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES manipulator_demo_module
#  CATKIN_DEPENDS robotis_device roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(manipulator_demo_module
  src/DemoModule.cpp
#  src/RobotisData.cpp
#  src/RobotisLink.cpp
  src/RobotisState.cpp
#  src/JointState.cpp
#  src/Transformation.cpp
#  src/Trajectory.cpp
  src/MoveitState.cpp
)

add_dependencies(manipulator_demo_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(manipulator_demo_module
  robotis_device
  robotis_math
  yaml-cpp
  manipulator_kinematics_dynamics
  ${catkin_LIBRARIES}
)
