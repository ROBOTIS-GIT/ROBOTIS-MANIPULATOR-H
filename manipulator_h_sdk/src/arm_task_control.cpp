#include <stdio.h>
#include "manipulator_h_sdk/arm_task_control.h"

using namespace manipulator_h;

TaskControl::TaskControl(std::string control_group,
                         double init_time, double fin_time,
                         geometry_msgs::Pose goal_msg)
{
  control_group_ = control_group;

  init_time_ = init_time;
  fin_time_ = fin_time;

  goal_msg_ = goal_msg;

  // Initialization
  init_arm_pos_.resize(3, 0.0);
  init_arm_vel_.resize(3, 0.0);
  init_arm_accel_.resize(3, 0.0);
  des_arm_pos_.resize(3, 0.0);
  des_arm_vel_.resize(3, 0.0);
  des_arm_accel_.resize(3, 0.0);
  goal_arm_pos_.resize(3, 0.0);
  goal_arm_vel_.resize(3, 0.0);
  goal_arm_accel_.resize(3, 0.0);

  goal_task_pos_.resize(3, 0.0);
  goal_task_vel_.resize(3, 0.0);
  goal_task_accel_.resize(3, 0.0);

  goal_task_pos_[0] = goal_msg_.position.x;
  goal_task_pos_[1] = goal_msg_.position.y;
  goal_task_pos_[2] = goal_msg_.position.z;

  Eigen::Quaterniond goal_task_Q(goal_msg_.orientation.w,goal_msg_.orientation.x,
                                 goal_msg_.orientation.y,goal_msg_.orientation.z);
  goal_task_Q_ = goal_task_Q;
}

TaskControl::~TaskControl()
{

}

void TaskControl::initialize(std::vector<double_t> init_arm_pos, std::vector<double_t> init_arm_Q)
{
  init_arm_pos_ = init_arm_pos;

  des_arm_pos_ = init_arm_pos_;

  Eigen::Quaterniond arm_Q(init_arm_Q[3],init_arm_Q[0],init_arm_Q[1],init_arm_Q[2]);
  init_arm_Q_ = arm_Q;
  des_arm_Q_ = arm_Q;

  if (control_group_ == "arm")
  {
    task_trajectory_ =
        new robotis_framework::MinimumJerk(init_time_, fin_time_,
                                           init_arm_pos_, init_arm_vel_, init_arm_accel_,
                                           goal_task_pos_, goal_task_vel_, goal_task_accel_);
    init_task_Q_ = arm_Q;
  }
}

void TaskControl::update()
{

}

void TaskControl::finalize()
{
  delete task_trajectory_;
}

void TaskControl::set(double time)
{
  std::vector<double_t> des_task_pos = task_trajectory_->getPosition(time);

  double count = time / fin_time_;
  des_task_Q_ = init_task_Q_.slerp(count, goal_task_Q_);

  if (control_group_ == "arm")
  {
    des_arm_pos_ = des_task_pos;
    des_arm_Q_ = des_task_Q_;
  }
}

void TaskControl::getTaskPosition(std::vector<double_t> &arm_pos)
{
  arm_pos = des_arm_pos_;
}

//std::vector<double_t> TaskControl::getTaskVelocity(std::vector<double_t> &arm_vel)
//{

//}

//std::vector<double_t> TaskControl::getTaskAcceleration(std::vector<double_t> &arm_accel)
//{

//}

void TaskControl::getTaskOrientation(std::vector<double_t> &arm_Q)
{
  arm_Q[0] = des_arm_Q_.x();
  arm_Q[1] = des_arm_Q_.y();
  arm_Q[2] = des_arm_Q_.z();
  arm_Q[3] = des_arm_Q_.w();
}
