cmake_minimum_required(VERSION 2.8.3)
project(manipulator_demo_module_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  JointPose.msg
  KinematicsPose.msg
  RobotisDemo.msg
)

add_service_files(
  FILES
  GetJointPose.srv
  GetKinematicsPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
