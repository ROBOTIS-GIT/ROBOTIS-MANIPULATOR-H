/*
 * RobotisManager.cpp
 *
 *  Created on: 2016. 1. 21.
 *      Author: zerom
 */


#include "robotis_controller/robotis_controller.h"

/* Motion Module Header */
#include "manipulator_h_demo_module/demo_module.h"

using namespace ROBOTIS;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Manipulator_Manager");
  ros::NodeHandle nh;

  ROS_INFO("manager->init");
  robotis_framework::RobotisController *controller = robotis_framework::RobotisController::getInstance();

  /* Load ROS Parameter */
  std::string offset_file = nh.param<std::string>("offset_table", "");
  std::string robot_file  = nh.param<std::string>("robot_file_path", "");

  std::string init_file   = nh.param<std::string>("init_file_path", "");

  /* gazebo simulation */
  controller->gazebo_mode_ = nh.param<bool>("gazebo", false);
  if (controller->gazebo_mode_ == true)
  {
    std::string robot_name = nh.param<std::string>("gazebo_robot_name", "");
    if (robot_name != "")
      controller->gazebo_robot_name_ = robot_name;
  }

  if (robot_file == "")
  {
    ROS_ERROR("NO robot file path in the ROS parameters.");
    return -1;
  }

  if (controller->initialize(robot_file, init_file) == false)
  {
    ROS_ERROR("ROBOTIS Controller Initialize Fail!");
    return -1;
  }

  if (offset_file != "")
    controller->loadOffset(offset_file);

  sleep(1);

  /* Add Motion Module */
  controller->addMotionModule((robotis_framework::MotionModule*) DemoModule::getInstance());

  controller->startTimer();

  while (ros::ok())
  {
    usleep(100);
  }

  return 0;
}
