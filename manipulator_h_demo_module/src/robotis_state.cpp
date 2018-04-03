
#include "manipulator_h_demo_module/robotis_state.h"

using namespace ROBOTIS;

namespace ROBOTIS_DEMO
{

RobotisState::RobotisState()
{
    is_moving = false;

    cnt = 0;

    mov_time = 1.0;
    smp_time = 0.02;
    all_time_steps = int( mov_time / smp_time ) + 1;

    calc_joint_tra = Eigen::MatrixXd::Zero( all_time_steps , MAX_JOINT_ID + 1 );
    calc_task_tra = Eigen::MatrixXd::Zero( all_time_steps , 3 );

    joint_ini_pose = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1 , 1 );

    via_num = 1;
    via_time = Eigen::MatrixXd::Zero( via_num , 1 );

    // for inverse kinematics;
    ik_solve = false;

    ik_target_position = robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );

    ik_start_rotation = robotis_framework::convertRPYToRotation( 0.0 , 0.0 , 0.0 );
    ik_target_rotation = robotis_framework::convertRPYToRotation( 0.0 , 0.0 , 0.0 );

    ik_id_start = 0;
    ik_id_end = 0;

    // msgs
    send_tra_msg.data = "";

    execute_planned_path = false;

}

RobotisState::~RobotisState(){}

void RobotisState::setInverseKinematics( int cnt ,Eigen::MatrixXd start_rotation )
{
    for ( int dim = 0; dim < 3; dim++ )
        ik_target_position.coeffRef( dim , 0 ) = calc_task_tra.coeff( cnt , dim );

    Eigen::Quaterniond _start_quaternion = robotis_framework::convertRotationToQuaternion( start_rotation );

    Eigen::Quaterniond _target_quaternion( kinematics_pose_msg.pose.orientation.w ,
                                           kinematics_pose_msg.pose.orientation.x ,
                                           kinematics_pose_msg.pose.orientation.y ,
                                           kinematics_pose_msg.pose.orientation.z );

    double _cnt = ( double ) cnt / ( double ) all_time_steps;

    Eigen::Quaterniond _quaternion = _start_quaternion.slerp( _cnt , _target_quaternion );

    ik_target_rotation = robotis_framework::convertQuaternionToRotation( _quaternion );
}


}
