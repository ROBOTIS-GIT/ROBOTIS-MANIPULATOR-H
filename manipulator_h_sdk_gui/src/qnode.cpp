
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include "manipulator_h_sdk_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace manipulator_h_sdk_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc_(argc),
  init_argv_(argv)
{}

QNode::~QNode() {
  if(ros::isStarted())
  {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init() {
  ros::init(init_argc_,init_argv_,"manipulator_h_sdk_gui");

  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;

  // Add your ros communications here.
  set_ctrl_module_pub_ = n.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);

  joint_pose_msg_pub_ = n.advertise<manipulator_h_sdk::JointPose>("/robotis/goal_joint_pose", 0);
  kinematics_pose_msg_pub_ = n.advertise<manipulator_h_sdk::KinematicsPose>("/robotis/goal_kinematics_pose", 0);

  get_joint_pose_client_ = n.serviceClient<manipulator_h_sdk::GetJointPose>("/robotis/get_joint_pose", 0);
  get_kinematics_pose_client_ = n.serviceClient<manipulator_h_sdk::GetKinematicsPose>("/robotis/get_kinematics_pose", 0);

  start();
  return true;
}

void QNode::run() {

  ros::Rate loop_rate(50);

  while ( ros::ok() )
  {
    ros::spinOnce();

    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::log( const LogLevel &level, const std::string &msg, std::string sender)
{
  logging_model_.insertRows(logging_model_.rowCount(),1);
  std::stringstream logging_model_msg;

  std::stringstream _sender;
  _sender << "[" << sender << "] ";

  switch ( level ) {
  case(Debug) : {
    ROS_DEBUG_STREAM(msg);
    logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << _sender.str() << msg;
    break;
  }
  case(Info) : {
    ROS_INFO_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << _sender.str() << msg;
    break;
  }
  case(Warn) : {
    ROS_WARN_STREAM(msg);
    logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  case(Error) : {
    ROS_ERROR_STREAM(msg);
    logging_model_msg << "<ERROR> [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  case(Fatal) : {
    ROS_FATAL_STREAM(msg);
    logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model_.setData(logging_model_.index(logging_model_.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::sendSetModeMsg()
{
  std_msgs::String str_msg;
  str_msg.data = "arm_control_module";

  set_ctrl_module_pub_.publish(str_msg);

  return;
}

void QNode::sendJointPoseMsg(manipulator_h_sdk::JointPose msg)
{
  joint_pose_msg_pub_.publish( msg );

  log( Info , "Send Joint Pose Msg" );
}

void QNode::sendKinematicsPoseMsg( manipulator_h_sdk::KinematicsPose msg )
{
  kinematics_pose_msg_pub_.publish( msg );

  log( Info , "Send Kinematics Pose Msg" );
}

void QNode::getJointPose()
{
  log( Info , "Get Current Joint Pose" );

  manipulator_h_sdk::GetJointPose get_joint_pose;

  // request

  // response
  if ( get_joint_pose_client_.call ( get_joint_pose ) )
  {
    manipulator_h_sdk::JointPose joint_pose;

    for ( int i = 0; i < get_joint_pose.response.pose.pose.name.size(); i++ )
    {
      joint_pose.pose.name.push_back( get_joint_pose.response.pose.pose.name[i] );
      joint_pose.pose.position.push_back( get_joint_pose.response.pose.pose.position[i] );
    }

    Q_EMIT updateCurrentJointPose( joint_pose );
  }
  else
    log(Error, "fail to get joint pose.");
}

void QNode::getKinematicsPose ( std::string group_name )
{
  log( Info , "Get Current Kinematics Pose" );

  manipulator_h_sdk::GetKinematicsPose get_kinematics_pose;

  // request
  get_kinematics_pose.request.name = group_name;

  // response
  if ( get_kinematics_pose_client_.call( get_kinematics_pose ) )
  {
    manipulator_h_sdk::KinematicsPose kinematcis_pose;

    kinematcis_pose.name = get_kinematics_pose.request.name;
    kinematcis_pose.pose = get_kinematics_pose.response.pose.pose;

    Q_EMIT updateCurrentKinematicsPose( kinematcis_pose );
  }
  else
    log(Error, "fail to get kinematcis pose.");
}

}  // namespace manipulator_h_sdk_gui
