cmake_minimum_required(VERSION 2.8.3)
project(robotis_manipulator_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
)

add_executable(manipulator_rviz_pub 
	src/manipulator_rviz_pub.cpp
)

target_link_libraries(manipulator_rviz_pub
  ${catkin_LIBRARIES}
)
