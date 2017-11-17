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

#include "manipulator_h_sdk/arm_control_module.h"

using namespace manipulator_h;

ArmControlModule::ArmControlModule()
  : control_cycle_sec_(0.008),
    is_moving_(false),
    goal_initialize_(false),
    joint_control_initialize_(false)
{
  enable_       = false;
  module_name_  = "arm_control_module";
  control_mode_ = robotis_framework::PositionControl;
  control_type_ = NONE;

  manipulator_h_tree_ = new ManipulatorH();

  result_["joint1"]  = new robotis_framework::DynamixelState();
  result_["joint2"]  = new robotis_framework::DynamixelState();
  result_["joint3"]  = new robotis_framework::DynamixelState();
  result_["joint4"]  = new robotis_framework::DynamixelState();
  result_["joint5"]  = new robotis_framework::DynamixelState();
  result_["joint6"]  = new robotis_framework::DynamixelState();

  joint_name_to_id_["joint1"]  = 1;
  joint_name_to_id_["joint2"]  = 2;
  joint_name_to_id_["joint3"]  = 3;
  joint_name_to_id_["joint4"]  = 4;
  joint_name_to_id_["joint5"]  = 5;
  joint_name_to_id_["joint6"]  = 6;

  joint_id_to_name_[1] = "joint1";
  joint_id_to_name_[2] = "joint2";
  joint_id_to_name_[3] = "joint3";
  joint_id_to_name_[4] = "joint4";
  joint_id_to_name_[5] = "joint5";
  joint_id_to_name_[6] = "joint6";

  /* parameter */
  number_of_joints_ = NUM_OF_JOINTS;

  pre_joint_accel_.resize(number_of_joints_, 0.0);
  pre_joint_vel_.resize(number_of_joints_, 0.0);
  pre_joint_pos_.resize(number_of_joints_, 0.0);

  des_joint_accel_.resize(number_of_joints_, 0.0);
  des_joint_vel_.resize(number_of_joints_, 0.0);
  des_joint_pos_.resize(number_of_joints_, 0.0);

  goal_joint_accel_.resize(number_of_joints_, 0.0);
  goal_joint_vel_.resize(number_of_joints_, 0.0);
  goal_joint_pos_.resize(number_of_joints_, 0.0);

  des_arm_pos_.resize(3, 0.0);
  des_arm_vel_.resize(3, 0.0);
  des_arm_accel_.resize(3, 0.0);
  des_arm_Q_.resize(4, 0.0);;

  tra_err_.resize(number_of_joints_, 0.0);
}

ArmControlModule::~ArmControlModule()
{
  queue_thread_.join();
}

void ArmControlModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_sec_ = control_cycle_msec * 0.001;
  queue_thread_      = boost::thread(boost::bind(&ArmControlModule::queueThread, this));

  ros::NodeHandle ros_node;

  // Publisher
  status_msg_pub_       = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
}

void ArmControlModule::queueThread()
{
  ros::NodeHandle     ros_node;
  ros::CallbackQueue  callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  // Subscriber
  ros::Subscriber joint_pose_sub_ = ros_node.subscribe("/robotis/goal_joint_pose", 5,
                                                       &ArmControlModule::goalJointPoseCallback, this);
  ros::Subscriber kinematics_pose_sub_ = ros_node.subscribe("/robotis/goal_kinematics_pose", 5,
                                                            &ArmControlModule::goalKinematicsPoseCallback, this);

  // Service
  ros::ServiceServer get_joint_pose_server = ros_node.advertiseService("/robotis/get_joint_pose",
                                                                       &ArmControlModule::getJointPoseCallback, this);
  ros::ServiceServer get_kinematics_pose_server = ros_node.advertiseService("/robotis/get_kinematics_pose",
                                                                            &ArmControlModule::getKinematicsPoseCallback, this);

  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void ArmControlModule::goalJointPoseCallback(const manipulator_h_sdk::JointPose& msg)
{
  if (enable_ == false)
    return;

  if (control_type_ == NONE || control_type_ == JOINT_CONTROL)
  {
    mov_time_ = msg.mov_time;

    for (size_t i = 0; i < msg.pose.name.size(); i++)
    {
      std::string joint_name = msg.pose.name[i];
      goal_joint_pos_[joint_name_to_id_[joint_name] - 1] = msg.pose.position[i];
    }

    joint_control_initialize_ = false;
    control_type_ = JOINT_CONTROL;
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

void ArmControlModule::initJointControl()
{
  if (joint_control_initialize_ == true)
    return;

  joint_control_initialize_ = true;

  double ini_time = 0.0;
  double mov_time = mov_time_;

  mov_step_ = 0;
  mov_size_ = (int) (mov_time / control_cycle_sec_) + 1;

  joint_tra_ =
      new robotis_framework::MinimumJerk(ini_time, mov_time,
                                         des_joint_pos_, des_joint_vel_, des_joint_accel_,
                                         goal_joint_pos_, goal_joint_vel_, goal_joint_accel_);

  tra_err_.resize(number_of_joints_, 0.0);

  if (is_moving_ == true)
    ROS_INFO("[UPDATE] Joint Control");
  else
  {
    is_moving_ = true;
    ROS_INFO("[START] Joint Control");
  }
}

void ArmControlModule::calcJointControl()
{
  if (is_moving_ == true)
  {
    double cur_time = (double) mov_step_ * control_cycle_sec_;

    queue_mutex_.lock();

    des_joint_pos_ = joint_tra_->getPosition(cur_time);
    des_joint_vel_ = joint_tra_->getVelocity(cur_time);
    des_joint_accel_ = joint_tra_->getAcceleration(cur_time);

    queue_mutex_.unlock();

    std::vector<double_t> err;
    err.resize(number_of_joints_, 0.0);

    for (int i=0; i<number_of_joints_; i++)
    {
      err[i] = fabs(des_joint_pos_[i] - pre_joint_pos_[i]);
      tra_err_[i] += err[i];
    }

    if (mov_step_ == mov_size_-1)
    {
      mov_step_ = 0;
      is_moving_ = false;
      delete joint_tra_;

      control_type_ = NONE;

      ROS_INFO("=== Trajectory Following Error ===");
      ROS_INFO("joint1: %f", tra_err_[0]);
      ROS_INFO("joint2: %f", tra_err_[1]);
      ROS_INFO("joint3: %f", tra_err_[2]);
      ROS_INFO("joint4: %f", tra_err_[3]);
      ROS_INFO("joint5: %f", tra_err_[4]);
      ROS_INFO("joint6: %f", tra_err_[5]);

      ROS_INFO("[END] Joint Control");
    }
    else
      mov_step_++;
  }
}

void ArmControlModule::goalKinematicsPoseCallback(const manipulator_h_sdk::KinematicsPose& msg)
{
  if (enable_ == false)
    return;

  if (control_type_ == NONE || control_type_ == TASK_CONTROL)
  {
    if (is_moving_ == true)
    {
      if (task_control_group_!=msg.name)
      {
        ROS_WARN("[WARN] Control group is different!");
        return;
      }
    }

    des_arm_pos_ = pre_arm_pos_;
    des_arm_Q_ = pre_arm_Q_;

    mov_time_ = msg.mov_time;
    task_control_group_ = msg.name;
    task_goal_msg_ = msg.pose;

    task_control_initialize_ = false;
    control_type_ = TASK_CONTROL;
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

void ArmControlModule::initTaskControl()
{
  if (task_control_initialize_ == true)
    return;

  task_control_initialize_ = true;

  double ini_time = 0.0;
  double mov_time = mov_time_;

  mov_step_ = 0;
  mov_size_ = (int) (mov_time / control_cycle_sec_) + 1;

  task_control_ =
      new TaskControl(task_control_group_,
                      ini_time, mov_time,
                      task_goal_msg_);

  if (is_moving_ == true)
  {
    // TODO
    // ROS_INFO("[UPDATE] Wholebody Control");
    // task_control_->update();
  }
  else
  {
    ROS_INFO("[START] Task Control");

    task_control_->initialize(des_arm_pos_, des_arm_Q_);
    is_moving_ = true;
  }
}

void ArmControlModule::calcTaskControl()
{
  if (is_moving_ == true)
  {
    double cur_time = (double) mov_step_ * control_cycle_sec_;

    task_control_->set(cur_time);

    task_control_->getTaskPosition(des_arm_pos_);
    task_control_->getTaskOrientation(des_arm_Q_);

    if (mov_step_ == mov_size_-1)
    {
      mov_step_ = 0;
      is_moving_ = false;
      task_control_->finalize();

      control_type_ = NONE;

      ROS_INFO("[END] Task Control");
    }
    else
      mov_step_++;
  }
}

void ArmControlModule::calcRobotPose()
{
  // Forward Kinematics
  manipulator_h_tree_->initialize();

  Eigen::VectorXd arm_joint_pos;

  arm_joint_pos.resize(number_of_joints_);
  arm_joint_pos(0) = des_joint_pos_[joint_name_to_id_["joint1"]-1];
  arm_joint_pos(1) = des_joint_pos_[joint_name_to_id_["joint2"]-1];
  arm_joint_pos(2) = des_joint_pos_[joint_name_to_id_["joint3"]-1];
  arm_joint_pos(3) = des_joint_pos_[joint_name_to_id_["joint4"]-1];
  arm_joint_pos(4) = des_joint_pos_[joint_name_to_id_["joint5"]-1];
  arm_joint_pos(5) = des_joint_pos_[joint_name_to_id_["joint6"]-1];

  manipulator_h_tree_->setJointPosition(arm_joint_pos);

  pre_arm_pos_.resize(3,0.0); pre_arm_Q_.resize(4,0.0);

  manipulator_h_tree_->solveForwardKinematics(pre_arm_pos_, pre_arm_Q_);

//  ROS_INFO("pre arm_pos x: %f, y: %f, z: %f", pre_arm_pos_[0], pre_arm_pos_[1], pre_arm_pos_[2]);

  if (control_type_ == TASK_CONTROL)
  {

    std::vector<double_t> arm_output;
    Eigen::MatrixXd des_arm_pos = Eigen::MatrixXd::Zero(3,1);

    for (int i=0; i<3; i++)
      des_arm_pos.coeffRef(i,0) = des_arm_pos_[i];

    Eigen::Quaterniond des_arm_Q(des_arm_Q_[3],des_arm_Q_[0],des_arm_Q_[1],des_arm_Q_[2]);

    bool ik_success;
    ik_success = manipulator_h_tree_->solveInverseKinematics(arm_output,
                                                              des_arm_pos,des_arm_Q);

    if (ik_success == true)
    {
      des_joint_pos_[joint_name_to_id_["joint1"]-1] = arm_output[0];
      des_joint_pos_[joint_name_to_id_["joint2"]-1] = arm_output[1];
      des_joint_pos_[joint_name_to_id_["joint3"]-1] = arm_output[2];
      des_joint_pos_[joint_name_to_id_["joint4"]-1] = arm_output[3];
      des_joint_pos_[joint_name_to_id_["joint5"]-1] = arm_output[4];
      des_joint_pos_[joint_name_to_id_["joint6"]-1] = arm_output[5];
    }
    else
    {
      mov_step_ = 0;
      is_moving_ = false;
      task_control_->finalize();

      control_type_ = NONE;

      ROS_ERROR("[END] Task Control : IK Failed");
    }
  }

  manipulator_h_tree_->finalize();
}

void ArmControlModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                              std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  /*----- write curr position -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    robotis_framework::Dynamixel *dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    double curr_joint_pos = dxl->dxl_state_->present_position_;
    double goal_joint_pos = dxl->dxl_state_->goal_position_;

    if (goal_initialize_ == false)
      des_joint_pos_[joint_name_to_id_[joint_name]-1] = goal_joint_pos;

    pre_joint_pos_[joint_name_to_id_[joint_name]-1] = curr_joint_pos;
  }

  goal_initialize_ = true;

  /* Trajectory Calculation */
  ros::Time begin = ros::Time::now();

  if (control_type_ == JOINT_CONTROL)
  {
    initJointControl();
    calcJointControl();
  }
  else if (control_type_ == TASK_CONTROL)
  {
    initTaskControl();
    calcTaskControl();
  }

  calcRobotPose();

  sensor_msgs::JointState goal_joint_msg;

  goal_joint_msg.header.stamp = ros::Time::now();
  /*----- set joint data -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_position_ = des_joint_pos_[joint_name_to_id_[joint_name]-1];
  }
}

void ArmControlModule::stop()
{
  is_moving_                  = false;
  goal_initialize_            = false;
  joint_control_initialize_   = false;

  control_type_ = NONE;

  return;
}

bool ArmControlModule::isRunning()
{
  return is_moving_;
}

void ArmControlModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status;
  status.header.stamp = ros::Time::now();
  status.type         = type;
  status.module_name  = "ArmControl";
  status.status_msg   = msg;

  status_msg_pub_.publish(status);
}

bool ArmControlModule::getJointPoseCallback(manipulator_h_sdk::GetJointPose::Request &req,
                                            manipulator_h_sdk::GetJointPose::Response &res)
{
  for (int i=0; i<number_of_joints_; i++)
  {
    res.pose.pose.name.push_back(joint_id_to_name_[i+1]);
    res.pose.pose.position.push_back(des_joint_pos_[i]);
  }

  return true;
}

bool ArmControlModule::getKinematicsPoseCallback(manipulator_h_sdk::GetKinematicsPose::Request &req,
                                                manipulator_h_sdk::GetKinematicsPose::Response &res)
{
  std::string group_name = req.name;

  geometry_msgs::Pose msg;

  if (group_name == "arm")
  {
    msg.position.x = pre_arm_pos_[0];
    msg.position.y = pre_arm_pos_[1];
    msg.position.z = pre_arm_pos_[2];

    msg.orientation.x = pre_arm_Q_[0];
    msg.orientation.y = pre_arm_Q_[1];
    msg.orientation.z = pre_arm_Q_[2];
    msg.orientation.w = pre_arm_Q_[3];
  }

  res.pose.pose = msg;

  return true;
}
