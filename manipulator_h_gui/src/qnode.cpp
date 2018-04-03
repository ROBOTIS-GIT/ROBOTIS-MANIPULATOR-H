/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include "../include/manipulator_h_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace manipulator_h_gui {

/*****************************************************************************
** Implementation
****************************f*************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv)
{}

QNode::~QNode() {
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init() {
  ros::init(init_argc,init_argv,"manipulator_h_gui");

  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;

  n.getParam("/manipulator_h_gui/ar_1_x_offset", ar_1_x_offset);
  n.getParam("/manipulator_h_gui/ar_1_y_offset", ar_1_y_offset);
  n.getParam("/manipulator_h_gui/ar_2_x_offset", ar_2_x_offset);
  n.getParam("/manipulator_h_gui/ar_2_y_offset", ar_2_y_offset);

  // Add your ros communications here.
  ini_pose_msg_pub = n.advertise<std_msgs::String>("/robotis/demo/ini_pose_msg", 0);
  set_mode_msg_pub = n.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);

  joint_pose_msg_pub = n.advertise<manipulator_h_demo_module_msgs::JointPose>("/robotis/demo/joint_pose_msg", 0);
  kinematics_pose_msg_pub = n.advertise<manipulator_h_demo_module_msgs::KinematicsPose>("/robotis/demo/kinematics_pose_msg", 0);

  execute_planned_path_msg_pub = n.advertise<std_msgs::String>("/robotis/demo/execute_planned_path", 0);

  robotis_demo_msg_pub = n.advertise<manipulator_h_demo_module_msgs::RobotisDemo>("/robotis/demo/robotis_demo_msg", 10, true);
  robotis_demo_msg_sub = n.subscribe("/robotis/demo/after_robotis_demo_msg", 10, &QNode::RobotisDemoMsgCallback, this);

  ar_1_pose_sub = n.subscribe("/ar_robotis_demo/1_pose", 10, &QNode::AR1PoseCallback, this);
  ar_2_pose_sub = n.subscribe("/ar_robotis_demo/2_pose", 10, &QNode::AR2PoseCallback, this);

  get_joint_pose_client = n.serviceClient<manipulator_h_demo_module_msgs::GetJointPose>("/robotis/demo/get_joint_pose", 0);
  get_kinematics_pose_client = n.serviceClient<manipulator_h_demo_module_msgs::GetKinematicsPose>("/robotis/demo/get_kinematics_pose", 0);

  current_text_pub = n.advertise<std_msgs::String>("/robotis/current_text", 0);

  syncwrite_item_pub_ = n.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);

  cnt = 0;

  ar_id = 0;
  send_ar_pose = false;
  get_ar_pose = false;
  half_cycle = false;

  get_ar_1_pose = false;
  get_ar_2_pose = false;

  is_demo_running = false;

  start();
  return true;
}

void QNode::run() {

  ros::Rate loop_rate(50);
  while ( ros::ok() )
  {
    ros::spinOnce();

    if ( send_ar_1_pose == true )
    {
      cnt++;

      if ( cnt > 2*tol_cnt )
        cnt = 0;

      if ( get_ar_1_pose == true )
      {
        get_ar_pose = false;
        manipulator_h_demo_module_msgs::RobotisDemo robotis_demo_msg;
        robotis_demo_msg.demo = "1_get_ar_pose";

        ar_pose_msg = ar_1_pose_msg;

        robotis_demo_msg.pose.position.x = ar_pose_msg.position.x;
        robotis_demo_msg.pose.position.y = ar_pose_msg.position.y;
        robotis_demo_msg.pose.position.z = 0.1;

        //                ROS_INFO("ar_1_pos_x : %f", robotis_demo_msg.pose.position.x );
        //                ROS_INFO("ar_1_pos_y : %f", robotis_demo_msg.pose.position.y );

        Eigen::Quaterniond _ar_pose_orientation( ar_pose_msg.orientation.w ,
                                                 ar_pose_msg.orientation.x ,
                                                 ar_pose_msg.orientation.y ,
                                                 ar_pose_msg.orientation.z );

        Eigen::MatrixXd _rpy = quaternion2rpy( _ar_pose_orientation );

        double _yaw = _rpy.coeff(2,0) * rad2deg;

        //                ROS_INFO("_yaw before : %f", _yaw);

        double tol = 90.0;

        if ( _yaw > tol )
          _yaw -= tol;
        else if ( _yaw < -tol )
          _yaw += tol;

        //                ROS_INFO("_yaw after : %f", _yaw);

        Eigen::Quaterniond _quaternion = rpy2quaternion( 0.0 , 90.0 * deg2rad , _yaw * deg2rad );

        robotis_demo_msg.pose.orientation.x = _quaternion.x();
        robotis_demo_msg.pose.orientation.y = _quaternion.y();
        robotis_demo_msg.pose.orientation.z = _quaternion.z();
        robotis_demo_msg.pose.orientation.w = _quaternion.w();

        robotis_demo_msg_pub.publish( robotis_demo_msg );

        send_ar_1_pose = false;

        ar_pose_msg = robotis_demo_msg.pose;

        std_msgs::String text_msg;
        text_msg.data = "Success...";
        current_text_pub.publish( text_msg );
      }
    }

    if ( send_ar_2_pose == true )
    {
      cnt++;

      if ( cnt > 2*tol_cnt )
        cnt = 0;

      if ( get_ar_2_pose == true )
      {
        get_ar_pose = false;
        manipulator_h_demo_module_msgs::RobotisDemo robotis_demo_msg;
        robotis_demo_msg.demo = "2_get_ar_pose";

        ar_pose_msg = ar_2_pose_msg;

        robotis_demo_msg.pose.position.x = ar_pose_msg.position.x;
        robotis_demo_msg.pose.position.y = ar_pose_msg.position.y;
        robotis_demo_msg.pose.position.z = 0.1;

        //                ROS_INFO("ar_2_pos_x : %f", robotis_demo_msg.pose.position.x );
        //                ROS_INFO("ar_2_pos_y : %f", robotis_demo_msg.pose.position.y );

        Eigen::Quaterniond _ar_pose_orientation( ar_pose_msg.orientation.w ,
                                                 ar_pose_msg.orientation.x ,
                                                 ar_pose_msg.orientation.y ,
                                                 ar_pose_msg.orientation.z );

        Eigen::MatrixXd _rpy = quaternion2rpy( _ar_pose_orientation );

        double _yaw = _rpy.coeff(2,0) * rad2deg;

        //                ROS_INFO("_yaw before : %f", _yaw);

        double tol = 90.0;

        if ( _yaw > tol )
          _yaw -= tol;
        else if ( _yaw < -tol )
          _yaw += tol;

        //                ROS_INFO("_yaw after : %f", _yaw);

        Eigen::Quaterniond _quaternion = rpy2quaternion( 0.0 , 90.0 * deg2rad , _yaw * deg2rad );

        robotis_demo_msg.pose.orientation.x = _quaternion.x();
        robotis_demo_msg.pose.orientation.y = _quaternion.y();
        robotis_demo_msg.pose.orientation.z = _quaternion.z();
        robotis_demo_msg.pose.orientation.w = _quaternion.w();

        robotis_demo_msg_pub.publish( robotis_demo_msg );

        send_ar_2_pose = false;

        ar_pose_msg = robotis_demo_msg.pose;

        std_msgs::String text_msg;
        text_msg.data = "Success...";
        current_text_pub.publish( text_msg );
      }
    }

    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;
  switch ( level ) {
  case(Debug) : {
    ROS_DEBUG_STREAM(msg);
    logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Info) : {
    ROS_INFO_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Warn) : {
    ROS_WARN_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Error) : {
    ROS_ERROR_STREAM(msg);
    logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Fatal) : {
    ROS_FATAL_STREAM(msg);
    logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::sendJointPoseMsg( manipulator_h_demo_module_msgs::JointPose msg )
{
  joint_pose_msg_pub.publish( msg );

  log( Info , "Send Joint Pose Msg" );
}

void QNode::sendKinematicsPoseMsg( manipulator_h_demo_module_msgs::KinematicsPose msg )
{
  kinematics_pose_msg_pub.publish( msg );

  log( Info , "Send Kinematics Pose Msg" );
}

void QNode::sendExecuteMsg( std_msgs::String msg )
{
  execute_planned_path_msg_pub.publish ( msg );

  log( Info , "Execute Planned Trajectory" );
}

void QNode::sendIniPoseMsg( std_msgs::String msg )
{
  ini_pose_msg_pub.publish ( msg );

  log( Info , "Go to Manipulator Initial Pose" );
}

void QNode::sendSetModeMsg( std_msgs::String msg )
{
  set_mode_msg_pub.publish ( msg );

  log( Info , "Set Demo Mode" );
}

void QNode::sendSyncSWriteItemMsg( robotis_controller_msgs::SyncWriteItem msg )
{
  syncwrite_item_pub_.publish (msg );

  log( Info , "Send SyncWrite Item" );
}

void QNode::getJointPose( std::vector<std::string> joint_name )
{
  log( Info , "Get Current Joint Pose" );

  manipulator_h_demo_module_msgs::GetJointPose _get_joint_pose;

  // request
  for ( int _id = 0; _id < joint_name.size(); _id++ )
    _get_joint_pose.request.joint_name.push_back( joint_name[ _id ] );

  // response
  if ( get_joint_pose_client.call ( _get_joint_pose ) )
  {
    manipulator_h_demo_module_msgs::JointPose _joint_pose;

    for ( int _id = 0; _id < _get_joint_pose.response.joint_name.size(); _id++ )
    {
      _joint_pose.name.push_back( _get_joint_pose.response.joint_name[ _id ] );
      _joint_pose.value.push_back( _get_joint_pose.response.joint_value[ _id ]);
    }

    Q_EMIT update_curr_joint_pose( _joint_pose );
  }
  else
    log(Error, "fail to get joint pose.");
}

void QNode::AR1PoseCallback( geometry_msgs::Pose ar_pose_msg )
{
  if ( get_ar_pose == true && cnt > tol_cnt )
  {
    log( Info , "[Phase 1] Success to Get AR 1 Pose" );
    ROS_INFO("Success to get ar_1_pose_msg");
    ar_1_pose_msg = ar_pose_msg;

    ROS_INFO("ar_1_x_offset : %f , ar_1_y_offset : %f", ar_1_x_offset , ar_1_y_offset);

    ar_1_pose_msg.position.x += ar_1_x_offset;
    ar_1_pose_msg.position.y += ar_1_y_offset;

    get_ar_1_pose = true;
  }
}

void QNode::AR2PoseCallback( geometry_msgs::Pose ar_pose_msg )
{
  if ( get_ar_pose == true && cnt > tol_cnt )
  {
    log( Info , "[Phase 2] Success to Get AR 2 Pose" );
    ROS_INFO("Success to get ar_2_pose_msg");
    ar_2_pose_msg = ar_pose_msg;

    ROS_INFO("ar_2_x_offset : %f , ar_2_y_offset : %f", ar_2_x_offset , ar_2_y_offset);

    ar_2_pose_msg.position.x += ar_2_x_offset;
    ar_2_pose_msg.position.y += ar_2_y_offset;

    get_ar_2_pose = true;
  }
}

void QNode::getKinematicsPose ( std::string group_name )
{
  log( Info , "Get Current Kinematics Pose" );

  manipulator_h_demo_module_msgs::GetKinematicsPose _get_kinematics_pose;

  // request
  _get_kinematics_pose.request.group_name = group_name;

  // response
  if ( get_kinematics_pose_client.call( _get_kinematics_pose ) )
  {
    manipulator_h_demo_module_msgs::KinematicsPose _kinematcis_pose;

    _kinematcis_pose.name = _get_kinematics_pose.request.group_name;
    _kinematcis_pose.pose = _get_kinematics_pose.response.group_pose;

    Q_EMIT update_curr_kinematics_pose( _kinematcis_pose );
  }
  else
    log(Error, "fail to get kinematcis pose.");
}

void QNode::sendRobotisDemoMsg(std_msgs::String msg )
{
  manipulator_h_demo_module_msgs::RobotisDemo robotis_demo_msg;

  if ( is_demo_running == true )
  {
    robotis_demo_msg.demo = msg.data;

    if ( msg.data == "1_go_ini_pose"  )
    {
      log( Info , "[Phase 1] Go Initial Pose" );

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOff();
    }
    else if ( msg.data == "1_go_base_pose" )
    {
      log( Info , "[Phase 1] Go Base Pose" );

      robotis_demo_msg.pose.position.x = 0.1;
      robotis_demo_msg.pose.position.y = 0.17;
      robotis_demo_msg.pose.position.z = 0.21;

      Eigen::Quaterniond _quaternion = rpy2quaternion( 0.0 , 67.5*deg2rad , 0.0 );

      robotis_demo_msg.pose.orientation.x = _quaternion.x();
      robotis_demo_msg.pose.orientation.y = _quaternion.y();
      robotis_demo_msg.pose.orientation.z = _quaternion.z();
      robotis_demo_msg.pose.orientation.w = _quaternion.w();

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOff();
    }
    else if ( msg.data == "1_get_ar_pose" )
    {
      log( Info , "[Phase 1] Get AR Marker 1 Pose" );

      send_ar_1_pose = true;
      get_ar_pose = true;
      get_ar_1_pose = false;
      cnt = 0;

      std_msgs::String text_msg;
      text_msg.data = "Recognizing...";
      current_text_pub.publish( text_msg );

      laserOn();
    }
    else if ( msg.data == "1_go_down" )
    {
      log( Info , "[Phase 1] Go Down" );

      robotis_demo_msg.pose = ar_pose_msg;

      robotis_demo_msg.pose.position.z = 0.06;

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOn();
    }
    else if ( msg.data == "1_grip_on" )
    {
      log( Info , "[Phase 1] Gripper On" );

      robotis_demo_msg.grip_val = 44.0;

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOn();
    }
    else if ( msg.data == "1_go_up")
    {
      log( Info , "[Phase 1] Go Up" );

      robotis_demo_msg.pose = ar_pose_msg;

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOn();
    }
    else if ( msg.data == "1_go_center" )
    {
      log( Info , "[Phase 1] Go Center" );

      robotis_demo_msg.pose.position.x = 0.2;
      robotis_demo_msg.pose.position.y = 0.0;
      robotis_demo_msg.pose.position.z = 0.2;

      Eigen::Quaterniond _quaternion = rpy2quaternion( 0.0 , 90.0*deg2rad , 0.0 );

      robotis_demo_msg.pose.orientation.x = _quaternion.x();
      robotis_demo_msg.pose.orientation.y = _quaternion.y();
      robotis_demo_msg.pose.orientation.z = _quaternion.z();
      robotis_demo_msg.pose.orientation.w = _quaternion.w();

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOn();
    }
    else if ( msg.data == "1_go_center_down" )
    {
      log( Info , "[Phase 1] Go Center Down" );

      robotis_demo_msg.pose.position.x = 0.2;
      robotis_demo_msg.pose.position.y = 0.0;
      robotis_demo_msg.pose.position.z = 0.099;

      Eigen::Quaterniond _quaternion = rpy2quaternion( 0.0 , 90.0*deg2rad , 0.0 );

      robotis_demo_msg.pose.orientation.x = _quaternion.x();
      robotis_demo_msg.pose.orientation.y = _quaternion.y();
      robotis_demo_msg.pose.orientation.z = _quaternion.z();
      robotis_demo_msg.pose.orientation.w = _quaternion.w();

      robotis_demo_msg_pub.publish( robotis_demo_msg );
    }
    else if ( msg.data == "1_grip_off" )
    {
      log( Info , "[Phase 1] Gripper Off" );

      robotis_demo_msg.grip_val = 10.0;

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOn();
    }
    else if ( msg.data == "2_go_ini_pose"  )
    {
      log( Info , "[Phase 2] Go Initial Pose" );

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOff();
    }
    else if ( msg.data == "2_go_base_pose" )
    {
      log( Info , "[Phase 2] Go Base Pose" );

      robotis_demo_msg.pose.position.x = 0.1;
      robotis_demo_msg.pose.position.y = -0.17;
      robotis_demo_msg.pose.position.z = 0.21;

      Eigen::Quaterniond _quaternion = rpy2quaternion( 0.0 , 67.5*deg2rad , 0.0 );

      robotis_demo_msg.pose.orientation.x = _quaternion.x();
      robotis_demo_msg.pose.orientation.y = _quaternion.y();
      robotis_demo_msg.pose.orientation.z = _quaternion.z();
      robotis_demo_msg.pose.orientation.w = _quaternion.w();

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOff();
    }
    else if ( msg.data == "2_get_ar_pose" )
    {
      log( Info , "[Phase 2] Get AR Marker 2 Pose" );

      send_ar_2_pose = true;
      get_ar_pose = true;
      get_ar_2_pose = false;
      cnt = 0;

      std_msgs::String text_msg;
      text_msg.data = "Recognizing...";
      current_text_pub.publish( text_msg );

      laserOn();
    }
    else if ( msg.data == "2_go_down" )
    {
      log( Info , "[Phase 2] Go Down" );

      robotis_demo_msg.pose = ar_pose_msg;

      robotis_demo_msg.pose.position.z = 0.06;

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOn();
    }
    else if ( msg.data == "2_grip_on" )
    {
      log( Info , "[Phase 2] Gripper On" );

      robotis_demo_msg.grip_val = 44.0;

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOn();
    }
    else if ( msg.data == "2_go_up")
    {
      log( Info , "[Phase 2] Go Up" );

      robotis_demo_msg.pose = ar_pose_msg;

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOn();
    }
    else if ( msg.data == "2_go_center" )
    {
      log( Info , "[Phase 2] Go Center" );

      robotis_demo_msg.pose.position.x = 0.2;
      robotis_demo_msg.pose.position.y = 0.0;
      robotis_demo_msg.pose.position.z = 0.2;

      Eigen::Quaterniond _quaternion = rpy2quaternion( 0.0 , 90.0*deg2rad , 0.0 );

      robotis_demo_msg.pose.orientation.x = _quaternion.x();
      robotis_demo_msg.pose.orientation.y = _quaternion.y();
      robotis_demo_msg.pose.orientation.z = _quaternion.z();
      robotis_demo_msg.pose.orientation.w = _quaternion.w();

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOn();
    }
    else if ( msg.data == "2_go_center_down" )
    {
      log( Info , "[Phase 2] Go Center Down" );

      robotis_demo_msg.pose.position.x = 0.2;
      robotis_demo_msg.pose.position.y = 0.0;
      robotis_demo_msg.pose.position.z = 0.15;

      Eigen::Quaterniond _quaternion = rpy2quaternion( 0.0 , 90.0*deg2rad , 0.0 );

      robotis_demo_msg.pose.orientation.x = _quaternion.x();
      robotis_demo_msg.pose.orientation.y = _quaternion.y();
      robotis_demo_msg.pose.orientation.z = _quaternion.z();
      robotis_demo_msg.pose.orientation.w = _quaternion.w();

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOn();
    }
    else if ( msg.data == "2_grip_off" )
    {
      log( Info , "[Phase 2] Gripper Off" );

      robotis_demo_msg.grip_val = 10.0;

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOn();
    }
    else if ( msg.data == "3_go_ini_pose"  )
    {
      log( Info , "[Phase 3] Go Initial Pose" );

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOff();
    }
    else if ( msg.data == "3_grip_on" )
    {
      log( Info , "[Phase 3] Gripper On" );

      robotis_demo_msg.grip_val = 60.0;

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOff();
    }
    else if ( msg.data == "3_go_clean_pose_1" )
    {
      log( Info , "[Phase 3] Go Clean Pose 1" );

      robotis_demo_msg.pose.position.x = 0.19;
      robotis_demo_msg.pose.position.y = 0.1;
      robotis_demo_msg.pose.position.z = 0.15;

      Eigen::Quaterniond _quaternion = rpy2quaternion( 0.0 , 90.0*deg2rad , 0.0 );

      robotis_demo_msg.pose.orientation.x = _quaternion.x();
      robotis_demo_msg.pose.orientation.y = _quaternion.y();
      robotis_demo_msg.pose.orientation.z = _quaternion.z();
      robotis_demo_msg.pose.orientation.w = _quaternion.w();

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOff();
    }
    else if ( msg.data == "3_clean_up_1" )
    {
      log( Info , "[Phase 3] Clean Up 1" );

      robotis_demo_msg.pose.position.x = 0.19;
      robotis_demo_msg.pose.position.y = -0.03;
      robotis_demo_msg.pose.position.z = 0.15;

      Eigen::Quaterniond _quaternion = rpy2quaternion( 0.0 , 90.0*deg2rad , 0.0 );

      robotis_demo_msg.pose.orientation.x = _quaternion.x();
      robotis_demo_msg.pose.orientation.y = _quaternion.y();
      robotis_demo_msg.pose.orientation.z = _quaternion.z();
      robotis_demo_msg.pose.orientation.w = _quaternion.w();

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOff();
    }
    else if ( msg.data == "3_go_clean_pose_2" )
    {
      log( Info , "[Phase 3] Go Clean Pose 2" );

      robotis_demo_msg.pose.position.x = 0.19;
      robotis_demo_msg.pose.position.y = -0.1;
      robotis_demo_msg.pose.position.z = 0.1;

      Eigen::Quaterniond _quaternion = rpy2quaternion( 0.0 , 90.0*deg2rad , 0.0 );

      robotis_demo_msg.pose.orientation.x = _quaternion.x();
      robotis_demo_msg.pose.orientation.y = _quaternion.y();
      robotis_demo_msg.pose.orientation.z = _quaternion.z();
      robotis_demo_msg.pose.orientation.w = _quaternion.w();

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOff();
    }
    else if ( msg.data == "3_clean_up_2" )
    {
      log( Info , "[Phase 3] Clean Up 2" );

      robotis_demo_msg.pose.position.x = 0.19;
      robotis_demo_msg.pose.position.y = 0.1;
      robotis_demo_msg.pose.position.z = 0.1;

      Eigen::Quaterniond _quaternion = rpy2quaternion( 0.0 , 90.0*deg2rad , 0.0 );

      robotis_demo_msg.pose.orientation.x = _quaternion.x();
      robotis_demo_msg.pose.orientation.y = _quaternion.y();
      robotis_demo_msg.pose.orientation.z = _quaternion.z();
      robotis_demo_msg.pose.orientation.w = _quaternion.w();

      robotis_demo_msg_pub.publish( robotis_demo_msg );

      laserOff();
    }
  }
}

void QNode::RobotisDemoMsgCallback( std_msgs::String after_robotis_demo_msg )
{
  std_msgs::String demo_msg;

  geometry_msgs::Pose empty_msg;

  if ( after_robotis_demo_msg.data == "1_go_ini_pose_end" )
    demo_msg.data = "1_go_base_pose";
  else if ( after_robotis_demo_msg.data == "1_go_base_pose_end" )
    demo_msg.data = "1_get_ar_pose";
  else if ( after_robotis_demo_msg.data == "1_get_ar_pose_end" )
    demo_msg.data = "1_go_down";
  else if ( after_robotis_demo_msg.data == "1_go_down_end" )
    demo_msg.data = "1_grip_on";
  else if ( after_robotis_demo_msg.data == "1_grip_on_end" )
    demo_msg.data = "1_go_up";
  else if ( after_robotis_demo_msg.data == "1_go_up_end" )
    demo_msg.data = "1_go_center";
  else if ( after_robotis_demo_msg.data == "1_go_center_end" )
    demo_msg.data = "1_go_center_down";
  else if ( after_robotis_demo_msg.data == "1_go_center_down_end" )
    demo_msg.data = "1_grip_off";
  else if ( after_robotis_demo_msg.data == "1_grip_off_end" )
  {
    demo_msg.data = "2_go_ini_pose";

    ar_pose_msg = empty_msg;
    ar_1_pose_msg = empty_msg;
  }
  else if ( after_robotis_demo_msg.data == "2_go_ini_pose_end" )
    demo_msg.data = "2_go_base_pose";
  else if ( after_robotis_demo_msg.data == "2_go_base_pose_end" )
    demo_msg.data = "2_get_ar_pose";
  else if ( after_robotis_demo_msg.data == "2_get_ar_pose_end" )
    demo_msg.data = "2_go_down";
  else if ( after_robotis_demo_msg.data == "2_go_down_end" )
    demo_msg.data = "2_grip_on";
  else if ( after_robotis_demo_msg.data == "2_grip_on_end" )
    demo_msg.data = "2_go_up";
  else if ( after_robotis_demo_msg.data == "2_go_up_end" )
    demo_msg.data = "2_go_center";
  else if ( after_robotis_demo_msg.data == "2_go_center_end" )
    demo_msg.data = "2_go_center_down";
  else if ( after_robotis_demo_msg.data == "2_go_center_down_end" )
    demo_msg.data = "2_grip_off";
  else if ( after_robotis_demo_msg.data == "2_grip_off_end" )
  {
    demo_msg.data = "3_go_ini_pose";

    ar_pose_msg = empty_msg;
    ar_2_pose_msg = empty_msg;
  }
  else if ( after_robotis_demo_msg.data == "3_go_ini_pose_end" )
    demo_msg.data = "3_grip_on";
  else if ( after_robotis_demo_msg.data == "3_grip_on_end" )
    demo_msg.data = "3_go_clean_pose_1";
  else if ( after_robotis_demo_msg.data == "3_go_clean_pose_1_end" )
    demo_msg.data = "3_clean_up_1";
  else if ( after_robotis_demo_msg.data == "3_clean_up_1_end" )
    demo_msg.data = "3_go_clean_pose_2";
  else if ( after_robotis_demo_msg.data == "3_go_clean_pose_2_end" )
    demo_msg.data = "3_clean_up_2";
  else if ( after_robotis_demo_msg.data == "3_clean_up_2_end" )
    demo_msg.data = "1_go_ini_pose";
  else if ( after_robotis_demo_msg.data == "1_get_ar_pose_fail" )
  {
    log( Info , "[Phase 1] Fail to Get AR Pose 1" );
    demo_msg.data = "1_go_ini_pose";

    ar_pose_msg = empty_msg;
    ar_1_pose_msg = empty_msg;

    send_ar_1_pose = false;
    get_ar_pose = false;
    get_ar_1_pose = false;
    cnt = 0;
  }
  else if ( after_robotis_demo_msg.data == "2_get_ar_pose_fail" )
  {
    log( Info , "[Phase 2] Fail to Get AR Pose 2" );
    demo_msg.data = "2_go_ini_pose";

    ar_pose_msg = empty_msg;
    ar_2_pose_msg = empty_msg;

    send_ar_2_pose = false;
    get_ar_pose = false;
    get_ar_2_pose = false;
    cnt = 0;
  }

  ros::Duration seconds(0.1);
  seconds.sleep();

  sendRobotisDemoMsg( demo_msg );
}

void QNode::laserOn()
{
  robotis_controller_msgs::SyncWriteItem msg, empty_msg;

  msg.item_name = "external_port_data_1";
  msg.joint_name.push_back("grip");
  msg.value.push_back(1);

  sendSyncSWriteItemMsg(msg);
  msg = empty_msg;

  msg.item_name = "external_port_data_2";
  msg.joint_name.push_back("grip");
  msg.value.push_back(1);

  sendSyncSWriteItemMsg(msg);
  msg = empty_msg;

  msg.item_name = "external_port_data_3";
  msg.joint_name.push_back("grip");
  msg.value.push_back(1);

  sendSyncSWriteItemMsg(msg);
  msg = empty_msg;

  msg.item_name = "external_port_data_4";
  msg.joint_name.push_back("grip");
  msg.value.push_back(1);

  sendSyncSWriteItemMsg(msg);
}

void QNode::laserOff()
{
  robotis_controller_msgs::SyncWriteItem msg, empty_msg;

  msg.item_name = "external_port_data_1";
  msg.joint_name.push_back("grip");
  msg.value.push_back(0);

  sendSyncSWriteItemMsg(msg);
  msg = empty_msg;

  msg.item_name = "external_port_data_2";
  msg.joint_name.push_back("grip");
  msg.value.push_back(0);

  sendSyncSWriteItemMsg(msg);
  msg = empty_msg;

  msg.item_name = "external_port_data_3";
  msg.joint_name.push_back("grip");
  msg.value.push_back(0);

  sendSyncSWriteItemMsg(msg);
  msg = empty_msg;

  msg.item_name = "external_port_data_4";
  msg.joint_name.push_back("grip");
  msg.value.push_back(0);

  sendSyncSWriteItemMsg(msg);
}

Eigen::MatrixXd QNode::rotationX( double angle )
{
  Eigen::MatrixXd _rotation( 3 , 3 );

  _rotation << 1.0,          0.0,           0.0,
      0.0, cos( angle ), -sin( angle ),
      0.0, sin( angle ),  cos( angle );

  return _rotation;
}

Eigen::MatrixXd QNode::rotationY( double angle )
{
  Eigen::MatrixXd _rotation( 3 , 3 );

  _rotation << cos( angle ), 0.0, sin( angle ),
      0.0, 1.0, 	     0.0,
      -sin( angle ), 0.0, cos( angle );

  return _rotation;
}

Eigen::MatrixXd QNode::rotationZ( double angle )
{
  Eigen::MatrixXd _rotation(3,3);

  _rotation << cos( angle ), -sin( angle ), 0.0,
      sin( angle ),  cos( angle ), 0.0,
      0.0,           0.0, 1.0;

  return _rotation;
}

Eigen::Quaterniond QNode::rpy2quaternion( double roll, double pitch, double yaw )
{
  Eigen::MatrixXd _rotation = rotationZ( yaw ) * rotationY( pitch ) * rotationX( roll );

  Eigen::Matrix3d _rotation3d;
  _rotation3d = _rotation.block( 0 , 0 , 3 , 3 );

  Eigen::Quaterniond _quaternion;

  _quaternion = _rotation3d;

  return _quaternion;
}

Eigen::MatrixXd QNode::quaternion2rpy( Eigen::Quaterniond quaternion )
{
  Eigen::MatrixXd _rotation = quaternion.toRotationMatrix() ;

  Eigen::MatrixXd _rpy = Eigen::MatrixXd::Zero( 3 , 1 );

  _rpy.coeffRef( 0 , 0 ) = atan2( _rotation.coeff( 2 , 1 ), _rotation.coeff( 2 , 2 ) );
  _rpy.coeffRef( 1 , 0 ) = atan2( -_rotation.coeff( 2 , 0 ), sqrt( pow( _rotation.coeff( 2 , 1 ) , 2 ) + pow( _rotation.coeff( 2 , 2 ) , 2 ) ) );
  _rpy.coeffRef( 2 , 0 ) = atan2 ( _rotation.coeff( 1 , 0 ) , _rotation.coeff( 0 , 0 ) );

  return _rpy;
}

}  // namespace manipulator_h_gui
