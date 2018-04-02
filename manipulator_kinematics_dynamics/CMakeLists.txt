cmake_minimum_required(VERSION 2.8.3)
project(manipulator_kinematics_dynamics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  robotis_math
  cmake_modules
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES manipulator_kinematics_dynamics
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(manipulator_kinematics_dynamics
  src/LinkData.cpp
  src/ManipulatorKinematicsDynamics.cpp
)

add_dependencies(manipulator_kinematics_dynamics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(manipulator_kinematics_dynamics
  ${catkin_LIBRARIES}
)
