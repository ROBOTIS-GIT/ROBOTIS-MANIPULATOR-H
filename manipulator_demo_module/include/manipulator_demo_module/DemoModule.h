/*
 * DemoModule.h
 *
 *  Created on: 2016. 3. 9.
 *      Author: sch
 */

#ifndef DEMOMODULE_H_
#define DEMOMODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"
#include "manipulator_kinematics_dynamics/ManipulatorKinematicsDynamics.h"

#include "robotis_controller_msgs/JointCtrlModule.h"

//#include "RobotisCommon.h"
//#include "RobotisData.h"
//#include "RobotisLink.h"
#include "RobotisState.h"
//#include "JointState.h"
//#include "Transformation.h"
//#include "Trajectory.h"
#include "MoveitState.h"

#include "manipulator_demo_module_msgs/JointPose.h"
#include "manipulator_demo_module_msgs/KinematicsPose.h"
#include "manipulator_demo_module_msgs/RobotisDemo.h"

#include "manipulator_demo_module_msgs/GetJointPose.h"
#include "manipulator_demo_module_msgs/GetKinematicsPose.h"

namespace ROBOTIS
{

//using namespace ROBOTIS_DEMO;

class DemoJointData
{

public:
  double position;
  double velocity;
  double effort;

  int p_gain;
  int i_gain;
  int d_gain;

};

class DemoJointState
{

public:
  DemoJointData curr_joint_state[ MAX_JOINT_ID + 1 ];
  DemoJointData goal_joint_state[ MAX_JOINT_ID + 1 ];
  DemoJointData fake_joint_state[ MAX_JOINT_ID + 1 ];

};

class DemoModule: public robotis_framework::MotionModule,
                  public robotis_framework::Singleton<DemoModule>
{
private:
//  static DemoModule *unique_instance_;

  int                 control_cycle_msec_;
  boost::thread       queue_thread_;
  boost::thread*      tra_gene_tread_;

  ros::Publisher      send_tra_pub_;
  ros::Publisher      set_ctrl_module_pub_;
  ros::Publisher      robotis_demo_msg_pub_;
  ros::Publisher      kinematics_pose_msg_pub_;

  ros::Publisher     current_text_pub_;

  std::map<std::string, int>  joint_name_to_id;

  bool moveit_execution;

  void QueueThread();

  void setCtrlModule(std::string module);
  void parseIniPoseData( const std::string &path );

public:
  DemoModule();
  virtual ~DemoModule();

  /* ROS Topic Callback Functions */
  void    IniPoseMsgCallback( const std_msgs::String::ConstPtr& msg );
  void    SetModeMsgCallback( const std_msgs::String::ConstPtr& msg );

  void    JointPoseMsgCallback( const manipulator_demo_module_msgs::JointPose::ConstPtr& msg );
  void    KinematicsPoseMsgCallback( const manipulator_demo_module_msgs::KinematicsPose::ConstPtr& msg );

  void    ExecutePlannedPathCallback( const std_msgs::String::ConstPtr& msg );

  bool    GetJointPoseCallback( manipulator_demo_module_msgs::GetJointPose::Request &req , manipulator_demo_module_msgs::GetJointPose::Response &res );
  bool    GetKinematicsPoseCallback( manipulator_demo_module_msgs::GetKinematicsPose::Request &req , manipulator_demo_module_msgs::GetKinematicsPose::Response &res );

  void    DisplayPlannedPathCallback( const moveit_msgs::DisplayTrajectory::ConstPtr& msg );

  void    RobotisDemoMsgCallback( const manipulator_demo_module_msgs::RobotisDemo::ConstPtr& msg );
  void    SendDemoEndMsg();

  /* ROS Calculation Functions */
  void    IniposeTraGeneProc( );
  void    JointTraGeneProc( );
  void    TaskTraGeneProc( );
  void    MoveitTraGeneProc( );
  void    GripTraGeneProc( );

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
  void stop();
  bool isRunning();



  /* Parameter */
  std_msgs::String  DemoName;
  double grip_val;

  ROBOTIS_DEMO::RobotisState *Robotis;
  ROBOTIS_DEMO::MoveitState *Moveit;

  ManipulatorKinematicsDynamics *ManipulatorH;
  DemoJointState *JointState;

};

}


#endif /* DEMOMODULE_H_ */
