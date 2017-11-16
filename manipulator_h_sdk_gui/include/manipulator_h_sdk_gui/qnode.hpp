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

/**
 * @file /include/manipulator_h_sdk_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef MANIPULATOR_H_SDK_GUI_QNODE_HPP_
#define MANIPULATOR_H_SDK_GUI_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#ifndef Q_MOC_RUN

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <string>
#include <boost/thread.hpp>
#include <QThread>
#include <QStringListModel>

#include <Eigen/Dense>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include <yaml-cpp/yaml.h>

#include "manipulator_h_sdk/JointPose.h"
#include "manipulator_h_sdk/KinematicsPose.h"

#include "manipulator_h_sdk/GetKinematicsPose.h"
#include "manipulator_h_sdk/GetJointPose.h"

#endif

#define DEG2RAD   (M_PI / 180.0)
#define RAD2DEG   (180.0 / M_PI)

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace manipulator_h_sdk_gui
{

/*****************************************************************************
 ** Class
 *****************************************************************************/

class QNode: public QThread
{
  Q_OBJECT
public:
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init();
  void run();

  /*********************
   ** Logging
   **********************/
  enum LogLevel
  {
    Debug, Info, Warn, Error, Fatal
  };

  QStringListModel* loggingModel()
  {
    return &logging_model_;
  }
  void log(const LogLevel &level, const std::string &msg, std::string sender = "GUI");

  void sendSetModeMsg();

  void sendJointPoseMsg(manipulator_h_sdk::JointPose msg);
  void sendKinematicsPoseMsg(manipulator_h_sdk::KinematicsPose msg);

public Q_SLOTS:
  void getJointPose();
  void getKinematicsPose(std::string group_name);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

  void updateCurrentJointPose(manipulator_h_sdk::JointPose);
  void updateCurrentKinematicsPose(manipulator_h_sdk::KinematicsPose);

private:
  int     init_argc_;
  char**  init_argv_;

  ros::Publisher      chatter_publisher_;
  QStringListModel    logging_model_;

  ros::Publisher      set_ctrl_module_pub_;
  ros::Publisher      joint_pose_msg_pub_;
  ros::Publisher      kinematics_pose_msg_pub_;

  ros::ServiceClient  get_joint_pose_client_;
  ros::ServiceClient  get_kinematics_pose_client_;

  ros::Subscriber     status_msg_sub_;

};

}  // namespace manipulator_h_sdk_gui

#endif /* MANIPULATOR_H_SDK_GUI_QNODE_HPP_ */
