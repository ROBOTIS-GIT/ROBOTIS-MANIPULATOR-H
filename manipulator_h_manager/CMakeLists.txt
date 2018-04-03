cmake_minimum_required(VERSION 2.8.3)
project(manipulator_h_manager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

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
  manipulator_h_demo_module
)

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES manipulator_h_manager
#  CATKIN_DEPENDS robotis_controller roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(manipulator_h_manager src/manipulator_h_manager.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(manipulator_h_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(manipulator_h_manager
  robotis_controller
  manipulator_h_demo_module
  ${catkin_LIBRARIES}
)
