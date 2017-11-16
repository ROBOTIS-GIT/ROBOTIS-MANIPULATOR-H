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

#ifndef MANIPULATOR_H_SDK_ARM_TASK_CONTROL_H_
#define MANIPULATOR_H_SDK_ARM_TASK_CONTROL_H_

#pragma once

#include <math.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <map>

#include <geometry_msgs/Pose.h>

#include "robotis_math/robotis_math.h"

namespace manipulator_h
{

class TaskControl
{
public:
  TaskControl(std::string control_group,
                   double init_time, double fin_time,
                   geometry_msgs::Pose goal_msg);
  virtual ~TaskControl();

  void initialize(std::vector<double_t> init_arm_pos, std::vector<double_t> init_arm_Q);
  void update();
  void finalize();

  void set(double time);

  void getTaskPosition(std::vector<double_t> &arm_pos);

  void getTaskOrientation(std::vector<double_t> &arm_Q);

private:
  robotis_framework::MinimumJerk *task_trajectory_;

  std::string control_group_;
  double init_time_, fin_time_;
  geometry_msgs::Pose goal_msg_;

  std::vector<double_t> init_arm_pos_, init_arm_vel_, init_arm_accel_;
  std::vector<double_t> des_arm_pos_, des_arm_vel_, des_arm_accel_;
  std::vector<double_t> goal_arm_pos_, goal_arm_vel_, goal_arm_accel_;
  Eigen::Quaterniond    init_arm_Q_, des_arm_Q_, goal_arm_Q_;

  std::vector<double_t> goal_task_pos_, goal_task_vel_, goal_task_accel_;
  Eigen::Quaterniond    init_task_Q_, des_task_Q_, goal_task_Q_;
};

}

#endif
