
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

#ifndef MANIPULATOR_H_SDK_ARM_KINEMATICS_H_
#define MANIPULATOR_H_SDK_ARM_KINEMATICS_H_

#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>

#include <math.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <map>

#include <geometry_msgs/Pose.h>

#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <Eigen/Dense>

#define NUM_OF_JOINTS (6)
#define D2R           (M_PI/180.0)

namespace manipulator_h
{

class ManipulatorH
{
public:
  ManipulatorH();
  virtual ~ManipulatorH();

  void initialize();
  void setJointPosition(Eigen::VectorXd arm_joint_position);
  void solveForwardKinematics(std::vector<double_t> &arm_position, std::vector<double_t> &arm_orientation);
  bool solveInverseKinematics(std::vector<double_t> &arm_output,
                              Eigen::MatrixXd arm_target_position, Eigen::Quaterniond arm_target_orientation);
  void finalize();

protected:
  KDL::ChainDynParam *arm_dyn_param_ = NULL;
  KDL::ChainJntToJacSolver *arm_jacobian_solver_;
  KDL::ChainFkSolverPos_recursive *arm_fk_solver_;
  KDL::ChainIkSolverVel_pinv *arm_ik_vel_solver_;
  KDL::ChainIkSolverPos_NR_JL *arm_ik_pos_solver_;

  Eigen::VectorXd arm_joint_position_;
  geometry_msgs::Pose arm_pose_;
};

}

#endif
