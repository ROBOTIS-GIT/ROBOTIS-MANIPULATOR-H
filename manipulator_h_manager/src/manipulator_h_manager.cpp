/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * RobotisManager.cpp
 *
 *  Created on: 2016. 1. 21.
 *      Author: zerom
 */

#include "manipulator_h_base_module/base_module.h"
#include "robotis_controller/robotis_controller.h"

/* Motion Module Header */

using namespace robotis_manipulator_h;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Robotis_Manipulator_H_Manager");
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
  controller->addMotionModule((robotis_framework::MotionModule*) BaseModule::getInstance());

  controller->startTimer();

  while (ros::ok())
  {
    usleep(100);
  }

  return 0;
}
