cmake_minimum_required(VERSION 2.8.3)
project(manipulator_h_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_math
  robotis_controller
  robotis_controller_msgs
  cmake_modules
  manipulator_h_kinematics_dynamics
  manipulator_h_base_module
)

find_package(Eigen REQUIRED)

catkin_package(
  LIBRARIES manipulator_h_manager
)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_executable(manipulator_h_manager src/manipulator_h_manager.cpp)
add_dependencies(manipulator_h_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manipulator_h_manager
  ${catkin_LIBRARIES}
)
