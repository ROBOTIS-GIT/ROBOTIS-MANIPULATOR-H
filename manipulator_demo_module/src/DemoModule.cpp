/*
 * ThorManipulation.cpp
 *
 *  Created on: 2016. 1. 18.
 *      Author: zerom
 */

#include <stdio.h>
#include "manipulator_demo_module/DemoModule.h"

using namespace ROBOTIS;

//int check_cnt = 0;

//DemoModule *DemoModule::unique_instance_ = new DemoModule();

DemoModule::DemoModule()
  : control_cycle_msec_(0.02)
{
  enable_          = false;
  module_name_     = "demo_module";
  control_mode_    = robotis_framework::PositionControl;
  result_["joint1"]  = new robotis_framework::DynamixelState();
  result_["joint2"]  = new robotis_framework::DynamixelState();
  result_["joint3"]  = new robotis_framework::DynamixelState();
  result_["joint4"]  = new robotis_framework::DynamixelState();
  result_["joint5"]  = new robotis_framework::DynamixelState();
  result_["joint6"]  = new robotis_framework::DynamixelState();
  result_["grip"]    = new robotis_framework::DynamixelState();

  joint_name_to_id["joint1"]  = 1;
  joint_name_to_id["joint2"]  = 2;
  joint_name_to_id["joint3"]  = 3;
  joint_name_to_id["joint4"]  = 4;
  joint_name_to_id["joint5"]  = 5;
  joint_name_to_id["joint6"]  = 6;
  joint_name_to_id["grip"]    = 7;

  DemoName.data = "";

  //    ManipulatorH = new ROBOTIS_DEMO::RobotisData( ROBOTIS_DEMO::ARM );
  Robotis = new ROBOTIS_DEMO::RobotisState();
  Moveit = new ROBOTIS_DEMO::MoveitState();

  ManipulatorH = new ManipulatorKinematicsDynamics( ARM );
  JointState = new DemoJointState();

  moveit_execution = false;
}

DemoModule::~DemoModule()
{
  queue_thread_.join();
}

void DemoModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&DemoModule::QueueThread, this));

  ros::NodeHandle _ros_node;

  /* publish topics */

  // for gui
  send_tra_pub_           = _ros_node.advertise<std_msgs::String>("/robotis/manipulation/send_tra", 1);
  set_ctrl_module_pub_	= _ros_node.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_ctrl_module", 1);

  robotis_demo_msg_pub_   = _ros_node.advertise<std_msgs::String>("/robotis/demo/after_robotis_demo_msg", 5, true );
  kinematics_pose_msg_pub_ = _ros_node.advertise<manipulator_demo_module_msgs::KinematicsPose>("/robotis/demo/kinematics_pose_msg", 5, true );

  current_text_pub_ = _ros_node.advertise<std_msgs::String>("/robotis/current_text", 0);

  Moveit->init( "robot_description" );
}

void DemoModule::parseIniPoseData( const std::string &path )
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile( path.c_str() );
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return ;
  }

  // parse movement time
  double _mov_time;
  _mov_time = doc["mov_time"].as< double >();

  Robotis->mov_time = _mov_time;

  // parse target pose
  YAML::Node _tar_pose_node = doc["tar_pose"];
  for(YAML::iterator _it = _tar_pose_node.begin() ; _it != _tar_pose_node.end() ; ++_it)
  {
    int _id;
    double _value;

    _id = _it->first.as<int>();
    _value = _it->second.as<double>();

    Robotis->joint_ini_pose.coeffRef( _id , 0 ) = _value * DEGREE2RADIAN;
  }

  Robotis->all_time_steps = int( Robotis->mov_time / Robotis->smp_time ) + 1;
  Robotis->calc_joint_tra.resize( Robotis->all_time_steps , MAX_JOINT_ID + 1 );
}

void DemoModule::QueueThread()
{
  ros::NodeHandle     _ros_node;
  ros::CallbackQueue  _callback_queue;

  _ros_node.setCallbackQueue(&_callback_queue);

  /* subscribe topics */

  // for gui
  ros::Subscriber ini_pose_msg_sub = _ros_node.subscribe("/robotis/demo/ini_pose_msg", 5, &DemoModule::IniPoseMsgCallback, this);
  ros::Subscriber set_mode_msg_sub = _ros_node.subscribe("/robotis/demo/set_mode_msg", 5, &DemoModule::SetModeMsgCallback, this);

  ros::Subscriber joint_pose_msg_sub = _ros_node.subscribe("/robotis/demo/joint_pose_msg", 5, &DemoModule::JointPoseMsgCallback, this);
  ros::Subscriber kinematics_pose_msg_sub = _ros_node.subscribe("/robotis/demo/kinematics_pose_msg", 5, &DemoModule::KinematicsPoseMsgCallback, this);

  ros::Subscriber execute_planned_path_sub = _ros_node.subscribe("robotis/demo/execute_planned_path", 5, &DemoModule::ExecutePlannedPathCallback, this);

  ros::ServiceServer get_joint_pose_server = _ros_node.advertiseService("/robotis/demo/get_joint_pose", &DemoModule::GetJointPoseCallback, this);
  ros::ServiceServer get_kinematics_pose_server = _ros_node.advertiseService("/robotis/demo/get_kinematics_pose", &DemoModule::GetKinematicsPoseCallback, this);

  /* moveit subscribe */
  //    ros::Subscriber joint_fake_states_sub 	 = nh.subscribe("/move_group/fake_controller_joint_states", 5, joint_fake_states_callback);
  ros::Subscriber display_planned_path_sub = _ros_node.subscribe("/move_group/display_planned_path", 		5, &DemoModule::DisplayPlannedPathCallback, this);

  /* demo */
  ros::Subscriber robotis_demo_msg_sub = _ros_node.subscribe("/robotis/demo/robotis_demo_msg", 5, &DemoModule::RobotisDemoMsgCallback, this);

  while(_ros_node.ok())
  {
    _callback_queue.callAvailable();
  }
}

void DemoModule::IniPoseMsgCallback( const std_msgs::String::ConstPtr& msg )
{
  if( enable_ == false )
    return;

  if ( Robotis->is_moving == false )
  {
    if ( msg->data == "ini_pose")
    {
      // parse initial pose
      std::string _ini_pose_path = ros::package::getPath("manipulator_demo_module") + "/config/ini_pose.yaml";
      parseIniPoseData( _ini_pose_path );

      tra_gene_tread_ = new boost::thread(boost::bind(&DemoModule::IniposeTraGeneProc, this));
      delete tra_gene_tread_;
    }
  }
  else
    ROS_INFO("previous task is alive");

  return;
}

void DemoModule::SetModeMsgCallback( const std_msgs::String::ConstPtr& msg )
{
  if ( enable_ == false )
  {
    if ( msg->data == "set_mode")
      setCtrlModule( module_name_ );
  }
  else
    ROS_INFO("previous task is alive");

  return;
}

bool DemoModule::GetJointPoseCallback( manipulator_demo_module_msgs::GetJointPose::Request &req , manipulator_demo_module_msgs::GetJointPose::Response &res )
{
  if ( enable_ == false )
    return false;

  for ( int _id = 1; _id <= MAX_JOINT_ID; _id++ )
  {
    for ( int _name_index = 0; _name_index < req.joint_name.size(); _name_index++ )
    {
      if( ManipulatorH->manipulator_link_data[ _id ]->name == req.joint_name[ _name_index ] )
      {
        res.joint_name.push_back( ManipulatorH->manipulator_link_data[ _id ]->name );
        res.joint_value.push_back( JointState->goal_joint_state[ _id ].position );

        break;
      }
    }
  }

  return true;
}

bool DemoModule::GetKinematicsPoseCallback( manipulator_demo_module_msgs::GetKinematicsPose::Request &req , manipulator_demo_module_msgs::GetKinematicsPose::Response &res )
{
  if ( enable_ == false )
    return false;

  //    int _end_index = 8;

  res.group_pose.position.x = Robotis->curr_position.coeff(0,0); //ManipulatorH->manipulator_link_data[ _end_index ]->position.coeff(0,0);
  res.group_pose.position.y = Robotis->curr_position.coeff(1,0); //ManipulatorH->manipulator_link_data[ _end_index ]->position.coeff(1,0);
  res.group_pose.position.z = Robotis->curr_position.coeff(2,0); //ManipulatorH->manipulator_link_data[ _end_index ]->position.coeff(2,0);

  //    Eigen::Quaterniond _quaternion = rotation2quaternion( ManipulatorH->manipulator_link_data[ _end_index ]->orientation );

  Eigen::Quaterniond _quaternion = robotis_framework::convertRotationToQuaternion( Robotis->curr_orientation );

  res.group_pose.orientation.w = _quaternion.w();
  res.group_pose.orientation.x = _quaternion.x();
  res.group_pose.orientation.y = _quaternion.y();
  res.group_pose.orientation.z = _quaternion.z();

  return  true;
}

void DemoModule::KinematicsPoseMsgCallback( const manipulator_demo_module_msgs::KinematicsPose::ConstPtr& msg )
{
  //    ROS_INFO("1");

  Robotis->kinematics_pose_msg = *msg;

  Robotis->ik_id_start = 0;
  Robotis->ik_id_end = 8;

  if ( Robotis->is_moving == false )
  {
    //        ROS_INFO("moveit execution : true");

    moveit_execution = true;

    //        tra_gene_tread_ = new boost::thread(boost::bind(&DemoModule::TaskTraGeneProc, this));
    //        delete tra_gene_tread_;
  }
  else
    ROS_INFO("previous task is alive");

  return;
}

void DemoModule::JointPoseMsgCallback( const manipulator_demo_module_msgs::JointPose::ConstPtr& msg )
{
  Robotis->joint_pose_msg = *msg;

  if ( Robotis->is_moving == false )
  {
    tra_gene_tread_ = new boost::thread(boost::bind(&DemoModule::JointTraGeneProc, this));
    delete tra_gene_tread_;
  }
  else
    ROS_INFO("previous task is alive");

  return;
}

void DemoModule::DisplayPlannedPathCallback( const moveit_msgs::DisplayTrajectory::ConstPtr& msg )
{
  //    ros::Time time = ros::Time::now();
  //    double _s = time.sec + time.nsec * 1e+9;
  //    ROS_INFO("Moveit Time [ %d ] : %f", check_cnt , _s);

  Moveit->moveit_msg = *msg;

  if ( Robotis->is_moving == false )
  {
    tra_gene_tread_ = new boost::thread(boost::bind(&DemoModule::MoveitTraGeneProc, this));
    delete tra_gene_tread_;
  }
  else
    ROS_INFO("Previous task is still alive!");

  return;
}

void DemoModule::ExecutePlannedPathCallback( const std_msgs::String::ConstPtr& msg )
{
  //    ros::Time time = ros::Time::now();
  //    double _s = time.sec + time.nsec * 1e+9;
  //    ROS_INFO("Bool Time [ %d ] : %f", check_cnt , _s);
  //    check_cnt++;

  if ( Robotis->is_moving == false )
  {
    if ( msg->data == "execute")
      Robotis->execute_planned_path = true;
    else if ( msg->data == "fail")
    {
      moveit_execution = false;

      ROS_INFO("Planning Fail");

      if ( DemoName.data == "1_get_ar_pose" )
        DemoName.data = "1_get_ar_pose_fail";
      else if ( DemoName.data == "2_get_ar_pose" )
        DemoName.data = "2_get_ar_pose_fail";

      robotis_demo_msg_pub_.publish( DemoName );
    }
  }

  return;
}

void DemoModule::RobotisDemoMsgCallback( const manipulator_demo_module_msgs::RobotisDemo::ConstPtr& msg )
{
  if ( Robotis->is_moving == false )
  {
    std_msgs::String text_msg;
    text_msg.data = "Planning...";
    current_text_pub_.publish( text_msg );

    DemoName.data = msg->demo;

    if ( msg->demo == "1_go_ini_pose" )
    {
      std::string _ini_pose_path = ros::package::getPath("manipulator_demo_module") + "/config/ini_pose.yaml";
      parseIniPoseData( _ini_pose_path );

      tra_gene_tread_ = new boost::thread(boost::bind(&DemoModule::IniposeTraGeneProc, this));
      delete tra_gene_tread_;
    }
    else if ( msg->demo == "1_go_base_pose" )
    {
      manipulator_demo_module_msgs::KinematicsPose kinematics_pose_msg;
      kinematics_pose_msg.pose = msg->pose;
      kinematics_pose_msg_pub_.publish( kinematics_pose_msg );

      moveit_execution = true;
    }
    else if ( msg->demo == "1_get_ar_pose" )
    {
      manipulator_demo_module_msgs::KinematicsPose kinematics_pose_msg;
      kinematics_pose_msg.pose = msg->pose;
      kinematics_pose_msg_pub_.publish( kinematics_pose_msg );

      moveit_execution = true;
    }
    else if ( msg->demo == "1_go_down")
    {
      manipulator_demo_module_msgs::KinematicsPose kinematics_pose_msg;
      kinematics_pose_msg.pose = msg->pose;
      kinematics_pose_msg_pub_.publish( kinematics_pose_msg );

      moveit_execution = true;
    }
    else if ( msg->demo == "1_grip_on" )
    {
      grip_val = msg->grip_val;

      tra_gene_tread_ = new boost::thread(boost::bind(&DemoModule::GripTraGeneProc, this));
      delete tra_gene_tread_;
    }
    else if ( msg->demo == "1_go_up")
    {
      manipulator_demo_module_msgs::KinematicsPose kinematics_pose_msg;
      kinematics_pose_msg.pose = msg->pose;
      kinematics_pose_msg_pub_.publish( kinematics_pose_msg );

      moveit_execution = true;
    }
    else if ( msg->demo == "1_go_center")
    {
      manipulator_demo_module_msgs::KinematicsPose kinematics_pose_msg;
      kinematics_pose_msg.pose = msg->pose;
      kinematics_pose_msg_pub_.publish( kinematics_pose_msg );

      moveit_execution = true;
    }
    else if ( msg->demo == "1_go_center_down")
    {
      manipulator_demo_module_msgs::KinematicsPose kinematics_pose_msg;
      kinematics_pose_msg.pose = msg->pose;
      kinematics_pose_msg_pub_.publish( kinematics_pose_msg );

      moveit_execution = true;
    }
    else if ( msg->demo == "1_grip_off" )
    {
      grip_val = msg->grip_val;

      tra_gene_tread_ = new boost::thread(boost::bind(&DemoModule::GripTraGeneProc, this));
      delete tra_gene_tread_;
    }
    else if ( msg->demo == "2_go_ini_pose" )
    {
      std::string _ini_pose_path = ros::package::getPath("manipulator_demo_module") + "/config/ini_pose.yaml";
      parseIniPoseData( _ini_pose_path );

      tra_gene_tread_ = new boost::thread(boost::bind(&DemoModule::IniposeTraGeneProc, this));
      delete tra_gene_tread_;
    }
    else if ( msg->demo == "2_go_base_pose" )
    {
      manipulator_demo_module_msgs::KinematicsPose kinematics_pose_msg;
      kinematics_pose_msg.pose = msg->pose;
      kinematics_pose_msg_pub_.publish( kinematics_pose_msg );

      moveit_execution = true;
    }
    else if ( msg->demo == "2_get_ar_pose" )
    {
      manipulator_demo_module_msgs::KinematicsPose kinematics_pose_msg;
      kinematics_pose_msg.pose = msg->pose;
      kinematics_pose_msg_pub_.publish( kinematics_pose_msg );

      moveit_execution = true;
    }
    else if ( msg->demo == "2_go_down")
    {
      manipulator_demo_module_msgs::KinematicsPose kinematics_pose_msg;
      kinematics_pose_msg.pose = msg->pose;
      kinematics_pose_msg_pub_.publish( kinematics_pose_msg );

      moveit_execution = true;
    }
    else if ( msg->demo == "2_grip_on" )
    {
      grip_val = msg->grip_val;

      tra_gene_tread_ = new boost::thread(boost::bind(&DemoModule::GripTraGeneProc, this));
      delete tra_gene_tread_;
    }
    else if ( msg->demo == "2_go_up")
    {
      manipulator_demo_module_msgs::KinematicsPose kinematics_pose_msg;
      kinematics_pose_msg.pose = msg->pose;
      kinematics_pose_msg_pub_.publish( kinematics_pose_msg );

      moveit_execution = true;
    }
    else if ( msg->demo == "2_go_center")
    {
      manipulator_demo_module_msgs::KinematicsPose kinematics_pose_msg;
      kinematics_pose_msg.pose = msg->pose;
      kinematics_pose_msg_pub_.publish( kinematics_pose_msg );

      moveit_execution = true;
    }
    else if ( msg->demo == "2_go_center_down")
    {
      manipulator_demo_module_msgs::KinematicsPose kinematics_pose_msg;
      kinematics_pose_msg.pose = msg->pose;
      kinematics_pose_msg_pub_.publish( kinematics_pose_msg );

      moveit_execution = true;
    }
    else if ( msg->demo == "2_grip_off" )
    {
      grip_val = msg->grip_val;

      tra_gene_tread_ = new boost::thread(boost::bind(&DemoModule::GripTraGeneProc, this));
      delete tra_gene_tread_;
    }
    else if ( msg->demo == "3_go_ini_pose" )
    {
      std::string _ini_pose_path = ros::package::getPath("manipulator_demo_module") + "/config/ini_pose.yaml";
      parseIniPoseData( _ini_pose_path );

      tra_gene_tread_ = new boost::thread(boost::bind(&DemoModule::IniposeTraGeneProc, this));
      delete tra_gene_tread_;
    }
    else if ( msg->demo == "3_grip_on" )
    {
      grip_val = msg->grip_val;

      tra_gene_tread_ = new boost::thread(boost::bind(&DemoModule::GripTraGeneProc, this));
      delete tra_gene_tread_;
    }
    else if ( msg->demo == "3_go_clean_pose_1" )
    {
      manipulator_demo_module_msgs::KinematicsPose kinematics_pose_msg;
      kinematics_pose_msg.pose = msg->pose;
      kinematics_pose_msg_pub_.publish( kinematics_pose_msg );

      moveit_execution = true;
    }
    else if ( msg->demo == "3_clean_up_1" )
    {
      manipulator_demo_module_msgs::KinematicsPose kinematics_pose_msg;
      kinematics_pose_msg.pose = msg->pose;
      kinematics_pose_msg_pub_.publish( kinematics_pose_msg );

      moveit_execution = true;
    }
    else if ( msg->demo == "3_go_clean_pose_2" )
    {
      manipulator_demo_module_msgs::KinematicsPose kinematics_pose_msg;
      kinematics_pose_msg.pose = msg->pose;
      kinematics_pose_msg_pub_.publish( kinematics_pose_msg );

      moveit_execution = true;
    }
    else if ( msg->demo == "3_clean_up_2" )
    {
      manipulator_demo_module_msgs::KinematicsPose kinematics_pose_msg;
      kinematics_pose_msg.pose = msg->pose;
      kinematics_pose_msg_pub_.publish( kinematics_pose_msg );

      moveit_execution = true;
    }
    else
      ROS_INFO("There is no demo");
  }
  else
    ROS_INFO("Previous task is still alive!");
}

void DemoModule::SendDemoEndMsg()
{
  if ( DemoName.data == "1_go_ini_pose" )
    DemoName.data = "1_go_ini_pose_end";
  else if ( DemoName.data == "1_go_base_pose" )
    DemoName.data = "1_go_base_pose_end";
  else if ( DemoName.data == "1_get_ar_pose" )
    DemoName.data = "1_get_ar_pose_end";
  else if ( DemoName.data == "1_go_down" )
    DemoName.data = "1_go_down_end";
  else if ( DemoName.data == "1_grip_on" )
    DemoName.data = "1_grip_on_end";
  else if ( DemoName.data == "1_go_up" )
    DemoName.data = "1_go_up_end";
  else if ( DemoName.data == "1_go_center" )
    DemoName.data = "1_go_center_end";
  else if ( DemoName.data == "1_go_center_down" )
    DemoName.data = "1_go_center_down_end";
  else if ( DemoName.data == "1_grip_off" )
    DemoName.data = "1_grip_off_end";
  else if ( DemoName.data == "2_go_ini_pose" )
    DemoName.data = "2_go_ini_pose_end";
  else if ( DemoName.data == "2_go_base_pose" )
    DemoName.data = "2_go_base_pose_end";
  else if ( DemoName.data == "2_get_ar_pose" )
    DemoName.data = "2_get_ar_pose_end";
  else if ( DemoName.data == "2_go_down" )
    DemoName.data = "2_go_down_end";
  else if ( DemoName.data == "2_grip_on" )
    DemoName.data = "2_grip_on_end";
  else if ( DemoName.data == "2_go_up" )
    DemoName.data = "2_go_up_end";
  else if ( DemoName.data == "2_go_center" )
    DemoName.data = "2_go_center_end";
  else if ( DemoName.data == "2_go_center_down" )
    DemoName.data = "2_go_center_down_end";
  else if ( DemoName.data == "2_grip_off" )
    DemoName.data = "2_grip_off_end";
  else if ( DemoName.data == "3_go_ini_pose" )
    DemoName.data = "3_go_ini_pose_end";
  else if ( DemoName.data == "3_grip_on" )
    DemoName.data = "3_grip_on_end";
  else if ( DemoName.data == "3_go_clean_pose_1" )
    DemoName.data = "3_go_clean_pose_1_end";
  else if ( DemoName.data == "3_clean_up_1" )
    DemoName.data = "3_clean_up_1_end";
  else if ( DemoName.data == "3_go_clean_pose_2" )
    DemoName.data = "3_go_clean_pose_2_end";
  else if ( DemoName.data == "3_clean_up_2" )
    DemoName.data = "3_clean_up_2_end";

  robotis_demo_msg_pub_.publish( DemoName );
}


void DemoModule::IniposeTraGeneProc()
{
  if( enable_ == false )
    return;

  for ( int id = 1; id <= MAX_JOINT_ID; id++ )
  {
    double ini_value = JointState->goal_joint_state[ id ].position;
    double tar_value = Robotis->joint_ini_pose.coeff( id , 0 );

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra( ini_value , 0.0 , 0.0 ,
                                                                 tar_value , 0.0 , 0.0 ,
                                                                 Robotis->smp_time , Robotis->mov_time );

    Robotis->calc_joint_tra.block( 0 , id , Robotis->all_time_steps , 1 ) = tra;
  }

  Robotis->cnt = 0;
  Robotis->is_moving = true;

  //    ROS_INFO("[start] send trajectory");
}

void DemoModule::JointTraGeneProc()
{
  if( enable_ == false )
    return;

  /* set movement time */
  double _tol = 35 * DEGREE2RADIAN; // rad per sec
  double _mov_time = 2.0;

  double _diff , __diff ;
  _diff = 0.0;

  for ( int _name_index = 0; _name_index < Robotis->joint_pose_msg.name.size(); _name_index++ )
  {
    double _ini_value;
    double _tar_value;

    for ( int _id = 1; _id <= MAX_JOINT_ID; _id++ )
    {
      if ( ManipulatorH->manipulator_link_data[ _id ]->name == Robotis->joint_pose_msg.name[ _name_index ] )
      {
        _ini_value = JointState->goal_joint_state[ _id ].position;
        _tar_value = Robotis->joint_pose_msg.value[ _name_index ];

        break;
      }
    }

    __diff = fabs ( _tar_value - _ini_value );

    if ( _diff < __diff )
      _diff = __diff;
  }

  Robotis->mov_time =  _diff / _tol;
  int _all_time_steps = int( floor( ( Robotis->mov_time / Robotis->smp_time ) + 1.0 ) );
  Robotis->mov_time = double ( _all_time_steps - 1 ) * Robotis->smp_time;

  if ( Robotis->mov_time < _mov_time )
    Robotis->mov_time = _mov_time;

  Robotis->all_time_steps = int( Robotis->mov_time / Robotis->smp_time ) + 1;

  Robotis->calc_joint_tra.resize( Robotis->all_time_steps , MAX_JOINT_ID + 1 );

  /* calculate joint trajectory */
  for ( int _id = 1; _id <= MAX_JOINT_ID; _id++ )
  {
    double ini_value = JointState->goal_joint_state[ _id ].position;
    double tar_value;

    for ( int _name_index = 0; _name_index < Robotis->joint_pose_msg.name.size(); _name_index++ )
    {
      if ( ManipulatorH->manipulator_link_data[ _id ]->name == Robotis->joint_pose_msg.name[ _name_index ] )
      {
        tar_value = Robotis->joint_pose_msg.value[ _name_index ];
        break;
      }
    }

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra( ini_value , 0.0 , 0.0 ,
                                                                 tar_value , 0.0 , 0.0 ,
                                                                 Robotis->smp_time , Robotis->mov_time );

    Robotis->calc_joint_tra.block( 0 , _id , Robotis->all_time_steps , 1 ) = tra;
  }

  Robotis->cnt = 0;
  Robotis->is_moving = true;

  //    ROS_INFO("[start] send trajectory");

}

void DemoModule::GripTraGeneProc()
{
  if( enable_ == false )
    return;

  /* set movement time */
  double _tol = 35 * DEGREE2RADIAN; // rad per sec
  double _mov_time = 2.0;

  double _ini_value = JointState->goal_joint_state[ 7 ].position;
  double _tar_value = grip_val * DEGREE2RADIAN;

  double _diff = fabs ( _tar_value - _ini_value );

  Robotis->mov_time =  _diff / _tol;
  int _all_time_steps = int( floor( ( Robotis->mov_time / Robotis->smp_time ) + 1.0 ) );
  Robotis->mov_time = double ( _all_time_steps - 1 ) * Robotis->smp_time;

  if ( Robotis->mov_time < _mov_time )
    Robotis->mov_time = _mov_time;

  Robotis->all_time_steps = int( Robotis->mov_time / Robotis->smp_time ) + 1;

  Robotis->calc_joint_tra.resize( Robotis->all_time_steps , MAX_JOINT_ID + 1 );

  /* calculate joint trajectory */
  for ( int _id = 1; _id <= MAX_JOINT_ID; _id++ )
  {
    double ini_value = JointState->goal_joint_state[ _id ].position;
    double tar_value = JointState->goal_joint_state[ _id ].position;;

    if ( _id == 7 )
      tar_value = grip_val * DEGREE2RADIAN;

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra( ini_value , 0.0 , 0.0 ,
                                                                 tar_value , 0.0 , 0.0 ,
                                                                 Robotis->smp_time , Robotis->mov_time );

    Robotis->calc_joint_tra.block( 0 , _id , Robotis->all_time_steps , 1 ) = tra;
  }

  Robotis->cnt = 0;
  Robotis->is_moving = true;

  //    ROS_INFO("[start] send trajectory");

}

//void DemoModule::TaskTraGeneProc()
//{
//    /* set movement time */
//    double _tol = 0.1; // m per sec
//    double _mov_time = 2.0;

//    double _diff = sqrt( pow( ManipulatorH->manipulator_link_data[ Robotis->ik_id_end ]->position.coeff( 0 , 0 ) - Robotis->kinematics_pose_msg.pose.position.x , 2  ) +
//                         pow( ManipulatorH->manipulator_link_data[ Robotis->ik_id_end ]->position.coeff( 1 , 0 ) - Robotis->kinematics_pose_msg.pose.position.y , 2  ) +
//                         pow( ManipulatorH->manipulator_link_data[ Robotis->ik_id_end ]->position.coeff( 2 , 0 ) - Robotis->kinematics_pose_msg.pose.position.z , 2  ) );

//    Robotis->mov_time = _diff / _tol;
//    int _all_time_steps = int( floor( ( Robotis->mov_time / Robotis->smp_time ) + 1.0 ) );
//    Robotis->mov_time = double ( _all_time_steps - 1 ) * Robotis->smp_time;

//    if ( Robotis->mov_time < _mov_time )
//        Robotis->mov_time = _mov_time;

//    Robotis->all_time_steps = int( Robotis->mov_time / Robotis->smp_time ) + 1;

//    Robotis->calc_task_tra.resize( Robotis->all_time_steps , 3 );

//     /* calculate trajectory */
//    for ( int dim = 0; dim < 3; dim++ )
//    {
//        double ini_value = ManipulatorH->manipulator_link_data[ Robotis->ik_id_end ]->position.coeff( dim , 0 );
//        double tar_value;
//        if ( dim == 0 )
//            tar_value = Robotis->kinematics_pose_msg.pose.position.x;
//        else if ( dim == 1 )
//            tar_value = Robotis->kinematics_pose_msg.pose.position.y;
//        else if ( dim == 2 )
//            tar_value = Robotis->kinematics_pose_msg.pose.position.z;

//        Eigen::MatrixXd tra = minimum_jerk_tra( ini_value , 0.0 , 0.0 ,
//                                                tar_value , 0.0 , 0.0 ,
//                                                Robotis->smp_time , Robotis->mov_time );

//        Robotis->calc_task_tra.block( 0 , dim , Robotis->all_time_steps , 1 ) = tra;
//    }

//    Robotis->cnt = 0;
//    Robotis->is_moving = true;
//    Robotis->ik_solve = true;

//    ROS_INFO("[start] send trajectory");
//}

void DemoModule::MoveitTraGeneProc()
{
  if( enable_ == false )
    return;

  //    ROS_INFO("[start] plan trajectory");

  std::vector<double> via_time;

  for ( int _tra_index = 0; _tra_index < Moveit->moveit_msg.trajectory.size(); _tra_index++ )
  {
    Moveit->points = Moveit->moveit_msg.trajectory[ _tra_index ].joint_trajectory.points.size();

    Moveit->display_planned_path_positions.resize	  ( Moveit->points , MAX_JOINT_ID + 1 );
    Moveit->display_planned_path_velocities.resize	  ( Moveit->points , MAX_JOINT_ID + 1 );
    Moveit->display_planned_path_accelerations.resize ( Moveit->points , MAX_JOINT_ID + 1 );

    for ( int _point_index = 0; _point_index < Moveit->points; _point_index++ )
    {
      for( int _name_index = 1; _name_index <= MAX_JOINT_ID; _name_index++ )
        Moveit->display_planned_path_positions.coeffRef( _point_index , _name_index ) = JointState->goal_joint_state[ _name_index ].position;
    }

    for ( int _point_index = 0; _point_index < Moveit->moveit_msg.trajectory[ _tra_index ].joint_trajectory.points.size(); _point_index++ )
    {
      Moveit->time_from_start = Moveit->moveit_msg.trajectory[ _tra_index ].joint_trajectory.points[_point_index].time_from_start;
      via_time.push_back( Moveit->time_from_start.toSec() );

      for ( int _joint_index = 0; _joint_index < Moveit->moveit_msg.trajectory[ _tra_index ].joint_trajectory.joint_names.size(); _joint_index++ )
      {
        std::string _joint_name 	= Moveit->moveit_msg.trajectory[ _tra_index ].joint_trajectory.joint_names[ _joint_index ];

        double _joint_position 		= Moveit->moveit_msg.trajectory[ _tra_index ].joint_trajectory.points[_point_index].positions	 [ _joint_index ];
        double _joint_velocity 	   	= Moveit->moveit_msg.trajectory[ _tra_index ].joint_trajectory.points[_point_index].velocities	 [ _joint_index ];
        double _joint_acceleration 	= Moveit->moveit_msg.trajectory[ _tra_index ].joint_trajectory.points[_point_index].accelerations[ _joint_index ];

        Moveit->display_planned_path_positions.coeffRef     ( _point_index , joint_name_to_id[ _joint_name ] ) = _joint_position;
        Moveit->display_planned_path_velocities.coeffRef	( _point_index , joint_name_to_id[ _joint_name ] ) = _joint_velocity;
        Moveit->display_planned_path_accelerations.coeffRef ( _point_index , joint_name_to_id[ _joint_name ] ) = _joint_acceleration;
      }
    }
  }

  Robotis->mov_time = Moveit->time_from_start.toSec();
  //    ROS_INFO("mov_time = %f", Robotis->mov_time);

  Robotis->all_time_steps = Moveit->points;
  //    ROS_INFO("all_time_steps = %d", Robotis->all_time_steps );

  //    ROS_INFO("[end] plan trajectory");

  //    PRINT_MAT( Moveit->display_planned_path_positions );

  ros::Duration seconds(0.5);
  seconds.sleep();

  //    if ( Robotis->execute_planned_path == true )
  if ( moveit_execution == true )
  {
    Robotis->calc_joint_tra = Moveit->display_planned_path_positions;

    Robotis->is_moving = true;
    Robotis->cnt = 0;

    moveit_execution = false;

    Robotis->execute_planned_path = false;

    //        ROS_INFO("[start] send trajectory");
  }
}

void DemoModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors)
{
  if(enable_ == false)
  {
    Robotis->is_moving = false;
    Robotis->ik_solve = false;
    Robotis->cnt = 0;

    return;
  }

  /*----- write curr position -----*/

  for(std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin(); state_iter != result_.end(); state_iter++)
  {
    std::string _joint_name = state_iter->first;

    robotis_framework::Dynamixel *_dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator _dxl_it = dxls.find(_joint_name);
    if(_dxl_it != dxls.end())
      _dxl = _dxl_it->second;
    else
      continue;

    double _joint_curr_position = _dxl->dxl_state_->present_position_;
    double _joint_goal_position = _dxl->dxl_state_->goal_position_;

    JointState->curr_joint_state[ joint_name_to_id[_joint_name] ].position = _joint_curr_position;
    JointState->goal_joint_state[ joint_name_to_id[_joint_name] ].position = _joint_goal_position;

    double _diff = fabs( _joint_goal_position - _joint_curr_position );

    if ( joint_name_to_id[_joint_name] != 7 )
    {
      if ( _diff > 30.0 * DEGREE2RADIAN )
      {
        ROS_INFO("JOINT ID : %d", joint_name_to_id[_joint_name] );
        ROS_INFO("PRESENT : %f", _joint_curr_position );
        ROS_INFO("GOAL : %f", _joint_goal_position );

        ROS_INFO("-----------------------");
        ROS_INFO("-----------------------");
        ROS_INFO("-----------------------");
        ROS_INFO("-----------------------");
        ROS_INFO("-----------------------");
        ROS_INFO("-----------------------");
        ROS_INFO("-----------------------");
        ROS_INFO("-----------------------");
        ROS_INFO("-----------------------");
        ROS_INFO("-----------------------");

        exit(1);
      }
    }
  }

  Moveit->kinematic_state->copyJointGroupPositions( Moveit->arm_joint_model_group , Moveit->_arm_joint_values );

  for ( int _arm_index = 1; _arm_index <= Moveit->_arm_joint_values.size(); _arm_index++ )
    Moveit->_arm_joint_values[ _arm_index - 1 ] = JointState->goal_joint_state[ _arm_index ].position;

  Moveit->kinematic_state->setJointGroupPositions( Moveit->arm_joint_model_group , Moveit->_arm_joint_values );

  const Eigen::Affine3d &curr_end_effector_state = Moveit->kinematic_state->getGlobalLinkTransform("end_effector");

  Robotis->curr_position =  curr_end_effector_state.translation();
  Robotis->curr_orientation =  curr_end_effector_state.rotation();

  /*----- forward kinematics -----*/

  //    for ( int id = 1; id <= MAX_JOINT_ID; id++ )
  //        ManipulatorH->manipulator_link_data[ id ]->joint_angle = JointState->goal_joint_state[ id ].position;

  //    ManipulatorH->forwardKinematics( 0 );



  /* ----- send trajectory ----- */

  if ( Robotis->is_moving == true )
  {
    if ( Robotis->cnt == 0 )
    {
      std_msgs::String text_msg;
      text_msg.data = "Moving...";
      current_text_pub_.publish( text_msg );

      //            Robotis->send_tra_msg.data = "start";
      //            send_tra_pub_.publish( Robotis->send_tra_msg );

      //            Robotis->ik_start_rotation = ManipulatorH->manipulator_link_data[ Robotis->ik_id_end ]->orientation;
    }

    //        if ( Robotis->cnt >= 0 && Robotis->cnt < 10 )
    //        {
    //            ROS_INFO("-----");
    //            ROS_INFO("cnt : %d/%d", Robotis->cnt , Robotis->all_time_steps );

    //            for ( int id = 1; id <= MAX_JOINT_ID; id++ )
    //            {
    //                ROS_INFO("dxl present[ %d ] : %f", id , JointState->curr_joint_state[ id ].position);
    //                ROS_INFO("dxl goal[ %d ] : %f", id , JointState->goal_joint_state[ id ].position);
    //            }
    //        }

    //        if ( Robotis->cnt > Robotis->all_time_steps - 10 && Robotis->cnt <= Robotis->all_time_steps )
    //        {
    //            ROS_INFO("-----");
    //            ROS_INFO("cnt : %d/%d", Robotis->cnt , Robotis->all_time_steps );
    //            for ( int id = 1; id <= MAX_JOINT_ID; id++ )
    //            {
    //                ROS_INFO("dxl present[ %d ] : %f", id , JointState->curr_joint_state[ id ].position);
    //                ROS_INFO("dxl goal[ %d ] : %f", id , JointState->goal_joint_state[ id ].position);
    //            }
    //        }

    //        if ( Robotis->ik_solve == true )
    //        {
    //            Robotis->setInverseKinematics( Robotis->cnt , Robotis->ik_start_rotation );

    //            int max_iter = 30;
    //            double ik_tol = 1e-3;
    //            bool ik_success = ManipulatorH->inverseKinematics( Robotis->ik_id_start , Robotis->ik_id_end ,
    //                                                               Robotis->ik_target_position , Robotis->ik_target_rotation ,
    //                                                               max_iter , ik_tol );

    //            if ( ik_success == true )
    //            {
    //                for ( int id = 1; id <= MAX_JOINT_ID; id++ )
    //                    JointState->goal_joint_state[ id ].position = ManipulatorH->manipulator_link_data[ id ]->joint_angle;
    //            }
    //            else
    //            {
    //                ROS_INFO("----- ik failed -----");
    //                ROS_INFO("[end] send trajectory");

    //                Robotis->send_tra_msg.data = "end";
    //                send_tra_pub_.publish( Robotis->send_tra_msg );

    //                Robotis->is_moving = false;
    //                Robotis->ik_solve = false;
    //                Robotis->cnt = 0;
    //            }
    //        }
    //        else
    //        {
    for ( int id = 1; id <= MAX_JOINT_ID; id++ )
    {
      JointState->goal_joint_state[ id ].position = Robotis->calc_joint_tra( Robotis->cnt , id );
    }

    //        ROS_INFO("cal goal[ 2 ] : %f" , JointState->goal_joint_state[ 2 ].position);

    //        if ( Robotis->cnt >= 0 && Robotis->cnt < 10 )
    //        {
    //            for ( int id = 1; id <= MAX_JOINT_ID; id++ )
    //                ROS_INFO("cal goal[ %d ] : %f", id , JointState->goal_joint_state[ id ].position);
    //        }

    //        if ( Robotis->cnt > Robotis->all_time_steps - 10 && Robotis->cnt <= Robotis->all_time_steps )
    //        {
    //            for ( int id = 1; id <= MAX_JOINT_ID; id++ )
    //                ROS_INFO("cal goal[ %d ] : %f", id , JointState->goal_joint_state[ id ].position);
    //        }

    //        }

    Robotis->cnt++;
  }



  /* ----- check self collision ----- */

  //    Moveit->kinematic_state->copyJointGroupPositions( Moveit->arm_joint_model_group , Moveit->arm_joint_values );

  //    for ( int _arm_index = 1; _arm_index <= Moveit->arm_joint_values.size(); _arm_index++ )
  //        Moveit->arm_joint_values[ _arm_index - 1 ] = JointState->goal_joint_state[ _arm_index ].position;

  //    collision_detection::CollisionRequest arm_collision_request;
  //    collision_detection::CollisionResult arm_collision_result;

  //    robot_state::RobotState& collision_state = Moveit->planning_scene->getCurrentStateNonConst();
  //    const robot_model::JointModelGroup* arm_collision_model_group = collision_state.getJointModelGroup("arm");

  //    arm_collision_request.group_name = "arm";
  //    collision_state.setJointGroupPositions( arm_collision_model_group , Moveit->arm_joint_values );
  //    Moveit->planning_scene->checkSelfCollision( arm_collision_request , arm_collision_result );

  //    if ( arm_collision_result.collision == true )
  //    {
  //        if ( arm_collision_result.collision == true )
  //            ROS_INFO_STREAM( "Next state will be "
  //                            << ( arm_collision_result.collision ? "in" : "not in" )
  //                            << " self collision" );

  //        for ( int _arm_index = 1; _arm_index <= Moveit->_arm_joint_values.size(); _arm_index++ )
  //            JointState->goal_joint_state[ _arm_index ].position = Moveit->_arm_joint_values[ _arm_index - 1 ];

  //        Robotis->send_tra_msg.data = "collision";
  //        send_tra_pub_.publish( Robotis->send_tra_msg );

  //        Robotis->is_moving = false;
  //        Robotis->ik_solve = false;
  //        Robotis->cnt = 0;
  //    }

  //    arm_collision_result.clear();

  /*----- set joint data -----*/

  //    ROS_INFO("-----");

  for(std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin(); state_iter != result_.end(); state_iter++)
  {
    std::string _joint_name = state_iter->first;
    result_[_joint_name]->goal_position_ = JointState->goal_joint_state[ joint_name_to_id[_joint_name] ].position;

    //        ROS_INFO("%s : %f", _joint_name.c_str() , result_[_joint_name]->goal_position );
  }

  //    ROS_INFO("-----");

  /*---------- initialize count number ----------*/

  if ( Robotis->is_moving == true )
  {
    if ( Robotis->cnt >= Robotis->all_time_steps )
    {
      //        ROS_INFO("[end] send trajectory");

      //        Robotis->send_tra_msg.data = "end";
      //        send_tra_pub_.publish( Robotis->send_tra_msg );

      Robotis->is_moving = false;
      Robotis->ik_solve = false;
      Robotis->cnt = 0;

      SendDemoEndMsg();
      DemoName.data = "";
    }
  }


}

void DemoModule::setCtrlModule(std::string module)
{
  robotis_controller_msgs::JointCtrlModule _control_msg;

  std::map<std::string, robotis_framework::DynamixelState *>::iterator _joint_iter;

  for(_joint_iter = result_.begin(); _joint_iter != result_.end(); ++_joint_iter)
  {
    _control_msg.joint_name.push_back( _joint_iter->first );
    _control_msg.module_name.push_back( module );
  }

  set_ctrl_module_pub_.publish(_control_msg);
}

void DemoModule::stop()
{
  return;
}

bool DemoModule::isRunning()
{
  return Robotis->is_moving;
}
