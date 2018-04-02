cmake_minimum_required(VERSION 2.8.3)
project(robotis_manipulator_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  std_msgs
)

add_executable(gazebo_grip_pub 
	src/gazebo_grip_pub.cpp
)

target_link_libraries(gazebo_grip_pub
  ${catkin_LIBRARIES}
)
