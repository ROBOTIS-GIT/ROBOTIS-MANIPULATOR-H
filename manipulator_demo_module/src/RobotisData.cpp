/*
 * Link.cpp
 *
 *  Created on: Jan 11, 2016
 *      Author: sch
 */

#include "manipulator_demo_module/RobotisCommon.h"
#include "manipulator_demo_module/RobotisLink.h"
#include "manipulator_demo_module/RobotisData.h"
#include "manipulator_demo_module/Transformation.h"

namespace ROBOTIS_DEMO
{

RobotisData::RobotisData() {}
RobotisData::~RobotisData() {}

RobotisData::RobotisData(TREE_SELECT tree)
{
    for ( int id = 0; id <= ALL_JOINT_ID; id++ )
        robotis_joint[ id ] = new RobotisLink();

    if ( tree == ARM )
    {
        robotis_joint[0]->name                  =   "base";
        robotis_joint[0]->parent                =   -1;
        robotis_joint[0]->sibling               =   -1;
        robotis_joint[0]->child                 =   1;
        robotis_joint[0]->mass                  =   0.0;
        robotis_joint[0]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[0]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[0]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[0]->joint_limit_max       =   100.0;
        robotis_joint[0]->joint_limit_min       =   -100.0;
        robotis_joint[0]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        robotis_joint[1]->name                  =   "joint1";
        robotis_joint[1]->parent                =   0;
        robotis_joint[1]->sibling               =   -1;
        robotis_joint[1]->child                 =   2;
        robotis_joint[1]->mass                  =   0.936;
        robotis_joint[1]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.126 );
        robotis_joint[1]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 1.0 );
        robotis_joint[1]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[1]->joint_limit_max       =   M_PI;
        robotis_joint[1]->joint_limit_min       =   -M_PI;
        robotis_joint[1]->inertia               =   inertiaXYZ( 1.0 , 0.0 , 0.0 , 1.0 , 0.0 , 1.0 );

        robotis_joint[2]->name                  =   "joint2";
        robotis_joint[2]->parent                =   1;
        robotis_joint[2]->sibling               =   -1;
        robotis_joint[2]->child                 =   3;
        robotis_joint[2]->mass                  =   1.030;
        robotis_joint[2]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.033 );
        robotis_joint[2]->joint_axis            =   transitionXYZ( 0.0 , 1.0 , 0.0 );
        robotis_joint[2]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[2]->joint_limit_max       =   M_PI;
        robotis_joint[2]->joint_limit_min       =   -M_PI;
        robotis_joint[2]->inertia               =   inertiaXYZ( 1.0 , 0.0 , 0.0 , 1.0 , 0.0 , 1.0 );

        robotis_joint[3]->name                  =   "joint3";
        robotis_joint[3]->parent                =   2;
        robotis_joint[3]->sibling               =   -1;
        robotis_joint[3]->child                 =   4;
        robotis_joint[3]->mass                  =   1.404;
        robotis_joint[3]->relative_position     =   transitionXYZ( 0.03 , 0.0 , 0.264 );
        robotis_joint[3]->joint_axis            =   transitionXYZ( 0.0 , 1.0 , 0.0 );
        robotis_joint[3]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[3]->joint_limit_max       =   M_PI;
        robotis_joint[3]->joint_limit_min       =   -M_PI;
        robotis_joint[3]->inertia               =   inertiaXYZ( 1.0 , 0.0 , 0.0 , 1.0 , 0.0 , 1.0 );

        robotis_joint[4]->name                  =   "joint4";
        robotis_joint[4]->parent                =   3;
        robotis_joint[4]->sibling               =   -1;
        robotis_joint[4]->child                 =   5;
        robotis_joint[4]->mass                  =   1.236;
        robotis_joint[4]->relative_position     =   rotationY( 90.0 * deg2rad) * transitionXYZ( -0.03 , 0.0 , 0.231 );
        robotis_joint[4]->joint_axis            =   rotationY( 90.0 * deg2rad) * transitionXYZ( 0.0 , 0.0 , 1.0 );
        robotis_joint[4]->center_of_mass        =   rotationY( 90.0 * deg2rad) * transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[4]->joint_limit_max       =   M_PI;
        robotis_joint[4]->joint_limit_min       =   -M_PI;
        robotis_joint[4]->inertia               =   inertiaXYZ( 1.0 , 0.0 , 0.0 , 1.0 , 0.0 , 1.0 );

        robotis_joint[5]->name                  =   "joint5";
        robotis_joint[5]->parent                =   4;
        robotis_joint[5]->sibling               =   -1;
        robotis_joint[5]->child                 =   6;
        robotis_joint[5]->mass                  =   0.491;
        robotis_joint[5]->relative_position     =   rotationY( 90.0 * deg2rad) * transitionXYZ( 0.0 , 0.0 , 0.027 );
        robotis_joint[5]->joint_axis            =   rotationY( 90.0 * deg2rad) * transitionXYZ( 0.0 , 1.0 , 0.0 );
        robotis_joint[5]->center_of_mass        =   rotationY( 90.0 * deg2rad) * transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[5]->joint_limit_max       =   M_PI;
        robotis_joint[5]->joint_limit_min       =   -M_PI;
        robotis_joint[5]->inertia               =   inertiaXYZ( 1.0 , 0.0 , 0.0 , 1.0 , 0.0 , 1.0 );

        robotis_joint[6]->name                  =   "joint6";
        robotis_joint[6]->parent                =   5;
        robotis_joint[6]->sibling               =   -1;
        robotis_joint[6]->child                 =   7;
        robotis_joint[6]->mass                  =   0.454;
        robotis_joint[6]->relative_position     =   rotationY( 90.0 * deg2rad) * transitionXYZ( 0.0 , 0.0 , 0.123 );
        robotis_joint[6]->joint_axis            =   rotationY( 90.0 * deg2rad) * transitionXYZ( 0.0 , 0.0 , 1.0 );
        robotis_joint[6]->center_of_mass        =   rotationY( 90.0 * deg2rad) * transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[6]->joint_limit_max       =   M_PI;
        robotis_joint[6]->joint_limit_min       =   -M_PI;
        robotis_joint[6]->inertia               =   inertiaXYZ( 1.0 , 0.0 , 0.0 , 1.0 , 0.0 , 1.0 );

        robotis_joint[7]->name                  =   "grip";
        robotis_joint[7]->parent                =   6;
        robotis_joint[7]->sibling               =   -1;
        robotis_joint[7]->child                 =   8;
        robotis_joint[7]->mass                  =   0.509;
        robotis_joint[7]->relative_position     =   rotationY( 90.0 * deg2rad) * transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[7]->joint_axis            =   rotationY( 90.0 * deg2rad) * transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[7]->center_of_mass        =   rotationY( 90.0 * deg2rad) * transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[7]->joint_limit_max       =   100;
        robotis_joint[7]->joint_limit_min       =   -100.0;
        robotis_joint[7]->inertia               =   inertiaXYZ( 1.0 , 0.0 , 0.0 , 1.0 , 0.0 , 1.0 );

        robotis_joint[8]->name                  =   "end";
        robotis_joint[8]->parent                =   7;
        robotis_joint[8]->sibling               =   -1;
        robotis_joint[8]->child                 =   -1;
        robotis_joint[8]->mass                  =   0.0;
        robotis_joint[8]->relative_position     =   rotationY( 90.0 * deg2rad) * transitionXYZ( 0.0 , 0.0 , 0.12 );
        robotis_joint[8]->joint_axis            =   rotationY( 90.0 * deg2rad) * transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[8]->center_of_mass        =   rotationY( 90.0 * deg2rad) * transitionXYZ( 0.0 , 0.0 , 0.0 );
        robotis_joint[8]->joint_limit_max       =   100.0;
        robotis_joint[8]->joint_limit_min       =   -100.0;
        robotis_joint[8]->inertia               =   inertiaXYZ( 1.0 , 0.0 , 0.0 , 1.0 , 0.0 , 1.0 );
    }
}

std::vector<int> RobotisData::findRoute( int to )
{
    int _id = robotis_joint[ to ]->parent;

    std::vector<int> _idx;

    if( _id == 0 )
    {
        _idx.push_back(0);
        _idx.push_back( to );
    }
    else
    {
        _idx = findRoute( _id );
        _idx.push_back( to );
    }

    return _idx;
}

std::vector<int> RobotisData::findRoute( int from , int to )
{
    int _id = robotis_joint[ to ]->parent;

    std::vector<int> _idx;

    if( _id == from )
    {
        _idx.push_back( from );
        _idx.push_back( to );
    }
    else if ( _id != 0 )
    {
        _idx = findRoute( from , _id );
        _idx.push_back( to );
    }

    return _idx;
}

double RobotisData::totalMass( int joint_ID )
{
    double _mass;

    if ( joint_ID == -1 )
        _mass = 0.0;
    else
        _mass = robotis_joint[ joint_ID ]->mass + totalMass( robotis_joint[ joint_ID ]->sibling ) + totalMass( robotis_joint[ joint_ID ]->child );

    return _mass;
}

Eigen::MatrixXd RobotisData::calcMC( int joint_ID )
{
    Eigen::MatrixXd _mc(3,1);

    if ( joint_ID == -1 )
        _mc = Eigen::MatrixXd::Zero(3,1);
    else
    {
        _mc = robotis_joint[ joint_ID ]->mass * ( robotis_joint[ joint_ID ]->orientation * robotis_joint[ joint_ID ]->center_of_mass + robotis_joint[ joint_ID ]->position );
        _mc = _mc + calcMC( robotis_joint[ joint_ID ]->sibling ) + calcMC( robotis_joint[ joint_ID ]->child );
    }

    return _mc;
}

Eigen::MatrixXd RobotisData::calcCOM( Eigen::MatrixXd MC )
{
    double _mass ;
    Eigen::MatrixXd _COM( 3 , 1 );

    _mass = totalMass( 0 );

    _COM = MC / _mass;

    return _COM;
}

void RobotisData::forwardKinematics( int joint_ID )
{
    if ( joint_ID == -1 )
        return;

//    robotis_joint[0]->position.block( 0 , 0 , 3 , 1 )       = Eigen::MatrixXd::Zero(3,1);
//    robotis_joint[0]->orientation.block( 0 , 0 , 3 , 3 )    = Rodrigues( hatto( robotis_joint[0]->joint_axis ), robotis_joint[ 0 ]->joint_angle );

//    robotis_joint[0]->position.block<3, 1>( 0 , 0)       = Eigen::MatrixXd::Zero(3,1);
//    robotis_joint[0]->orientation.block<3, 3>( 0 , 0)    = Rodrigues( hatto( robotis_joint[0]->joint_axis ), robotis_joint[ 0 ]->joint_angle );

    robotis_joint[0]->position       = Eigen::MatrixXd::Zero(3,1);
    robotis_joint[0]->orientation    = Rodrigues( hatto( robotis_joint[0]->joint_axis ), robotis_joint[ 0 ]->joint_angle );

    if ( joint_ID != 0 )
    {
        int _parent = robotis_joint[ joint_ID ]->parent;

//        robotis_joint[ joint_ID ]->position.block( 0 , 0 , 3 , 1 ) =
//                robotis_joint[ _parent ]->orientation * robotis_joint[ joint_ID ]->relative_position + robotis_joint[ _parent ]->position;
//        robotis_joint[ joint_ID ]->orientation.block( 0 , 0 , 3 , 3 ) =
//                robotis_joint[ _parent ]->orientation * Rodrigues( hatto( robotis_joint[ joint_ID ]->joint_axis ), robotis_joint[ joint_ID ]->joint_angle );
//
//        robotis_joint[ joint_ID ]->transformation.block ( 0 , 3 , 3 , 1 ) = robotis_joint[ joint_ID ]->position;
//        robotis_joint[ joint_ID ]->transformation.block ( 0 , 0 , 3 , 3 ) = robotis_joint[ joint_ID ]->orientation;

//        robotis_joint[ joint_ID ]->position.block<3, 1>( 0 , 0 ) =
//                robotis_joint[ _parent ]->orientation * robotis_joint[ joint_ID ]->relative_position + robotis_joint[ _parent ]->position;
//        robotis_joint[ joint_ID ]->orientation.block<3, 3>( 0 , 0 ) =
//                robotis_joint[ _parent ]->orientation * Rodrigues( hatto( robotis_joint[ joint_ID ]->joint_axis ), robotis_joint[ joint_ID ]->joint_angle );
//
//        robotis_joint[ joint_ID ]->transformation.block<3, 1> ( 0 , 3) = robotis_joint[ joint_ID ]->position;
//        robotis_joint[ joint_ID ]->transformation.block<3, 3> ( 0 , 0) = robotis_joint[ joint_ID ]->orientation;

        robotis_joint[ joint_ID ]->position =
                robotis_joint[ _parent ]->orientation * robotis_joint[ joint_ID ]->relative_position + robotis_joint[ _parent ]->position;
        robotis_joint[ joint_ID ]->orientation =
                robotis_joint[ _parent ]->orientation * Rodrigues( hatto( robotis_joint[ joint_ID ]->joint_axis ), robotis_joint[ joint_ID ]->joint_angle );

        robotis_joint[ joint_ID ]->transformation.block<3, 1> ( 0 , 3) = robotis_joint[ joint_ID ]->position;
        robotis_joint[ joint_ID ]->transformation.block<3, 3> ( 0 , 0) = robotis_joint[ joint_ID ]->orientation;
    }

    forwardKinematics( robotis_joint[ joint_ID ]->sibling );
    forwardKinematics( robotis_joint[ joint_ID ]->child );
}

Eigen::MatrixXd RobotisData::calcJacobian( std::vector<int> idx )
{
    int _idx_size 	= 	idx.size();
    int _end 		= 	_idx_size - 1;

    Eigen::MatrixXd _tar_position    = 	robotis_joint[ idx[ _end ] ]->position;
    Eigen::MatrixXd _Jacobian        = 	Eigen::MatrixXd::Zero( 6 , _idx_size );

    for ( int id = 0; id < _idx_size; id++)
    {
        int _id 					= 	idx[ id ];

        Eigen::MatrixXd _tar_orientation      = 	robotis_joint[ _id ]->orientation * robotis_joint[ _id ]->joint_axis;

        _Jacobian.block( 0 , id , 3 , 1 ) 	= 	cross( _tar_orientation , _tar_position - robotis_joint[ _id ]->position );
        _Jacobian.block( 3 , id , 3 , 1 ) 	= 	_tar_orientation;
    }

    return _Jacobian;
}

Eigen::MatrixXd RobotisData::calcJacobianCOM(std::vector<int> idx)
{
    int _idx_size	=	idx.size();
    int _end 		=	_idx_size - 1;

    Eigen::MatrixXd _target_position    = 	robotis_joint[ idx[ _end ] ]->position;
    Eigen::MatrixXd _jacobianCOM        = 	Eigen::MatrixXd::Zero( 6 , _idx_size );

    for ( int id = 0; id < _idx_size; id++ )
    {
        int _id 			= 	idx[ id ];
        double _mass 	= 	totalMass( _id );

        Eigen::MatrixXd _og                     =	calcMC( _id ) / _mass - robotis_joint[ _id ]->position;
        Eigen::MatrixXd _target_orientation 	= 	robotis_joint[ _id ]->orientation * robotis_joint[ _id ]->joint_axis;

        _jacobianCOM.block( 0 , id , 3 , 1 ) 	= 	cross( _target_orientation , _og );
        _jacobianCOM.block( 3 , id , 3 , 1 ) 	= 	_target_orientation;
    }

    return _jacobianCOM;
}

Eigen::MatrixXd RobotisData::calcVWerr( Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position, Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation )
{
    Eigen::MatrixXd _pos_err        =	tar_position - curr_position;
    Eigen::MatrixXd _ori_err        =	curr_orientation.inverse() * tar_orientation;
    Eigen::MatrixXd __ori_err      =	curr_orientation * rot2omega( _ori_err );

    Eigen::MatrixXd _err 	= 	Eigen::MatrixXd::Zero( 6 , 1 );
    _err.block(0,0,3,1) 		= 	_pos_err;
    _err.block(3,0,3,1) 		= 	__ori_err;

    return _err;
}

bool RobotisData::inverseKinematics( int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err )
{
    bool ik_success = false;
    bool limit_success = false;

    forwardKinematics(0);

    std::vector<int> _idx   = 	findRoute( to );

    for ( int iter = 0; iter < max_iter; iter++ )
    {
        Eigen::MatrixXd _Jacobian		= 	calcJacobian( _idx );

        Eigen::MatrixXd _curr_position      = 	robotis_joint[ to ]->position;
        Eigen::MatrixXd _curr_orientation 	= 	robotis_joint[ to ]->orientation;

        Eigen::MatrixXd _err 	= 	calcVWerr( tar_position,_curr_position, tar_orientation, _curr_orientation );

        if ( _err.norm() < ik_err )
        {
            ik_success = true;
            break;
        }
        else
            ik_success = false;


        Eigen::MatrixXd __Jacobian      = 	_Jacobian * _Jacobian.transpose();
        Eigen::MatrixXd ___Jacobian     = 	_Jacobian.transpose() * __Jacobian.inverse();

        Eigen::MatrixXd _delta_angle 		= 	___Jacobian * _err ;

        for ( int id = 0; id < _idx.size(); id++ )
        {
            int _joint_num      = 	_idx[ id ];
            robotis_joint[ _joint_num ]->joint_angle    += 	_delta_angle.coeff( id );
        }

        forwardKinematics(0);
    }

    for ( int id = 0; id < _idx.size(); id++ )
    {
        int _joint_num      = 	_idx[ id ];

        if ( robotis_joint[ _joint_num ]->joint_angle >= robotis_joint[ _joint_num ]->joint_limit_max )
        {
            limit_success = false;
            break;
        }
        else if ( robotis_joint[ _joint_num ]->joint_angle <= robotis_joint[ _joint_num ]->joint_limit_min )
        {
            limit_success = false;
            break;
        }
        else
            limit_success = true;
    }

    if ( ik_success == true && limit_success == true )
        return true;
    else
        return false;
}

bool RobotisData::inverseKinematics( int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err )
{
    bool ik_success = false;
    bool limit_success = false;

    forwardKinematics(0);

    std::vector<int> _idx   = 	findRoute( from , to );

    for ( int iter = 0; iter < max_iter; iter++ )
    {
        Eigen::MatrixXd _Jacobian		= 	calcJacobian( _idx );

        Eigen::MatrixXd _curr_position      = 	robotis_joint[ to ]->position;
        Eigen::MatrixXd _curr_orientation 	= 	robotis_joint[ to ]->orientation;

        Eigen::MatrixXd _err 	= 	calcVWerr( tar_position,_curr_position, tar_orientation, _curr_orientation );

        if ( _err.norm() < ik_err )
        {
            ik_success = true;
            break;
        }
        else
            ik_success = false;


        Eigen::MatrixXd __Jacobian      = 	_Jacobian * _Jacobian.transpose();
        Eigen::MatrixXd ___Jacobian     = 	_Jacobian.transpose() * __Jacobian.inverse();

        Eigen::MatrixXd _delta_angle 		= 	___Jacobian * _err ;

        for ( int id = 0; id < _idx.size(); id++ )
        {
            int _joint_num      = 	_idx[ id ];
            robotis_joint[ _joint_num ]->joint_angle    += 	_delta_angle.coeff( id );
        }

        forwardKinematics(0);
    }

    for ( int id = 0; id < _idx.size(); id++ )
    {
        int _joint_num      = 	_idx[ id ];

        if ( robotis_joint[ _joint_num ]->joint_angle >= robotis_joint[ _joint_num ]->joint_limit_max )
        {
            limit_success = false;
            break;
        }
        else if ( robotis_joint[ _joint_num ]->joint_angle <= robotis_joint[ _joint_num ]->joint_limit_min )
        {
            limit_success = false;
            break;
        }
        else
            limit_success = true;
    }

    if ( ik_success == true && limit_success == true )
        return true;
    else
        return false;
}

}
