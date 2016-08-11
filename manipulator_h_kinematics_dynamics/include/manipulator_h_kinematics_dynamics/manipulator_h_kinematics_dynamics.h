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

#ifndef MANIPULATOR_KINEMATICS_DYNAMICS_MANIPULATOR_KINEMATICS_DYNAMICS_H_
#define MANIPULATOR_KINEMATICS_DYNAMICS_MANIPULATOR_KINEMATICS_DYNAMICS_H_

#include <vector>

#include "link_data.h"
#include "manipulator_h_kinematics_dynamics_define.h"

namespace robotis_manipulator_h
{

enum TreeSelect
{
  ARM
};

class ManipulatorKinematicsDynamics
{
public:
  ManipulatorKinematicsDynamics();
  ManipulatorKinematicsDynamics(TreeSelect tree);
  ~ManipulatorKinematicsDynamics();

  LinkData *manipulator_link_data_[ ALL_JOINT_ID + 1];

  std::vector<int> findRoute(int to);
  std::vector<int> findRoute(int from, int to);

  double totalMass(int joint_ID);
  Eigen::MatrixXd calcMC(int joint_ID);
  Eigen::MatrixXd calcCOM(Eigen::MatrixXd MC);

  void forwardKinematics(int joint_ID);

  Eigen::MatrixXd calcJacobian(std::vector<int> idx);
  Eigen::MatrixXd calcJacobianCOM(std::vector<int> idx);
  Eigen::MatrixXd calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position,
                            Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation);

  bool inverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                         int max_iter, double ik_err);
  bool inverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                         int max_iter, double ik_err);
};

}

#endif /* MANIPULATOR_KINEMATICS_DYNAMICS_MANIPULATOR_KINEMATICS_DYNAMICS_H_ */
