cmake_minimum_required(VERSION 2.8.3)
project(manipulator_h_base_module)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  robotis_device
  robotis_math
  robotis_controller_msgs
  robotis_framework_common
  roscpp
  std_msgs
  geometry_msgs
  manipulator_h_kinematics_dynamics
  manipulator_h_base_module_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES manipulator_h_base_module
#  CATKIN_DEPENDS robotis_device roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(manipulator_h_base_module
  src/base_module.cpp
  src/robotis_state.cpp
)

add_dependencies(manipulator_h_base_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(manipulator_h_base_module
  yaml-cpp
  ${catkin_LIBRARIES}
)
