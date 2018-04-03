/*
 * Link.cpp
 *
 *  Created on: Jan 11, 2016
 *      Author: sch
 */


#include "manipulator_h_kinematics_dynamics/manipulator_h_kinematics_dynamics.h"
#include <iostream>

namespace ROBOTIS
{

ManipulatorKinematicsDynamics::ManipulatorKinematicsDynamics() {}
ManipulatorKinematicsDynamics::~ManipulatorKinematicsDynamics() {}

ManipulatorKinematicsDynamics::ManipulatorKinematicsDynamics(TREE_SELECT tree)
{
    for ( int id = 0; id <= ALL_JOINT_ID; id++ )
        manipulator_link_data[ id ] = new LinkData();

    if ( tree == ARM )
    {
        manipulator_link_data[0]->name                  =   "base";
        manipulator_link_data[0]->parent                =   -1;
        manipulator_link_data[0]->sibling               =   -1;
        manipulator_link_data[0]->child                 =   1;
        manipulator_link_data[0]->mass                  =   0.0;
        manipulator_link_data[0]->relative_position     =   robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
        manipulator_link_data[0]->joint_axis            =   robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
        manipulator_link_data[0]->center_of_mass        =   robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
        manipulator_link_data[0]->joint_limit_max       =   100.0;
        manipulator_link_data[0]->joint_limit_min       =   -100.0;
        manipulator_link_data[0]->inertia               =   robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        manipulator_link_data[1]->name                  =   "joint1";
        manipulator_link_data[1]->parent                =   0;
        manipulator_link_data[1]->sibling               =   -1;
        manipulator_link_data[1]->child                 =   2;
        manipulator_link_data[1]->mass                  =   0.936;
        manipulator_link_data[1]->relative_position     =   robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.126 );
        manipulator_link_data[1]->joint_axis            =   robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 1.0 );
        manipulator_link_data[1]->center_of_mass        =   robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
        manipulator_link_data[1]->joint_limit_max       =   M_PI;
        manipulator_link_data[1]->joint_limit_min       =   -M_PI;
        manipulator_link_data[1]->inertia               =   robotis_framework::getInertiaXYZ( 1.0 , 0.0 , 0.0 , 1.0 , 0.0 , 1.0 );

        manipulator_link_data[2]->name                  =   "joint2";
        manipulator_link_data[2]->parent                =   1;
        manipulator_link_data[2]->sibling               =   -1;
        manipulator_link_data[2]->child                 =   3;
        manipulator_link_data[2]->mass                  =   1.030;
        manipulator_link_data[2]->relative_position     =   robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.033 );
        manipulator_link_data[2]->joint_axis            =   robotis_framework::getTransitionXYZ( 0.0 , 1.0 , 0.0 );
        manipulator_link_data[2]->center_of_mass        =   robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
        manipulator_link_data[2]->joint_limit_max       =   M_PI;
        manipulator_link_data[2]->joint_limit_min       =   -M_PI;
        manipulator_link_data[2]->inertia               =   robotis_framework::getInertiaXYZ( 1.0 , 0.0 , 0.0 , 1.0 , 0.0 , 1.0 );

        manipulator_link_data[3]->name                  =   "joint3";
        manipulator_link_data[3]->parent                =   2;
        manipulator_link_data[3]->sibling               =   -1;
        manipulator_link_data[3]->child                 =   4;
        manipulator_link_data[3]->mass                  =   1.404;
        manipulator_link_data[3]->relative_position     =   robotis_framework::getTransitionXYZ( 0.03 , 0.0 , 0.264 );
        manipulator_link_data[3]->joint_axis            =   robotis_framework::getTransitionXYZ( 0.0 , 1.0 , 0.0 );
        manipulator_link_data[3]->center_of_mass        =   robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
        manipulator_link_data[3]->joint_limit_max       =   M_PI;
        manipulator_link_data[3]->joint_limit_min       =   -M_PI;
        manipulator_link_data[3]->inertia               =   robotis_framework::getInertiaXYZ( 1.0 , 0.0 , 0.0 , 1.0 , 0.0 , 1.0 );

        manipulator_link_data[4]->name                  =   "joint4";
        manipulator_link_data[4]->parent                =   3;
        manipulator_link_data[4]->sibling               =   -1;
        manipulator_link_data[4]->child                 =   5;
        manipulator_link_data[4]->mass                  =   1.236;
        manipulator_link_data[4]->relative_position     =   robotis_framework::getRotationY( 90.0 * DEGREE2RADIAN) * robotis_framework::getTransitionXYZ( -0.03 , 0.0 , 0.231 );
        manipulator_link_data[4]->joint_axis            =   robotis_framework::getRotationY( 90.0 * DEGREE2RADIAN) * robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 1.0 );
        manipulator_link_data[4]->center_of_mass        =   robotis_framework::getRotationY( 90.0 * DEGREE2RADIAN) * robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
        manipulator_link_data[4]->joint_limit_max       =   M_PI;
        manipulator_link_data[4]->joint_limit_min       =   -M_PI;
        manipulator_link_data[4]->inertia               =   robotis_framework::getInertiaXYZ( 1.0 , 0.0 , 0.0 , 1.0 , 0.0 , 1.0 );

        manipulator_link_data[5]->name                  =   "joint5";
        manipulator_link_data[5]->parent                =   4;
        manipulator_link_data[5]->sibling               =   -1;
        manipulator_link_data[5]->child                 =   6;
        manipulator_link_data[5]->mass                  =   0.491;
        manipulator_link_data[5]->relative_position     =   robotis_framework::getRotationY( 90.0 * DEGREE2RADIAN) * robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.027 );
        manipulator_link_data[5]->joint_axis            =   robotis_framework::getRotationY( 90.0 * DEGREE2RADIAN) * robotis_framework::getTransitionXYZ( 0.0 , 1.0 , 0.0 );
        manipulator_link_data[5]->center_of_mass        =   robotis_framework::getRotationY( 90.0 * DEGREE2RADIAN) * robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
        manipulator_link_data[5]->joint_limit_max       =   M_PI;
        manipulator_link_data[5]->joint_limit_min       =   -M_PI;
        manipulator_link_data[5]->inertia               =   robotis_framework::getInertiaXYZ( 1.0 , 0.0 , 0.0 , 1.0 , 0.0 , 1.0 );

        manipulator_link_data[6]->name                  =   "joint6";
        manipulator_link_data[6]->parent                =   5;
        manipulator_link_data[6]->sibling               =   -1;
        manipulator_link_data[6]->child                 =   7;
        manipulator_link_data[6]->mass                  =   0.454;
        manipulator_link_data[6]->relative_position     =   robotis_framework::getRotationY( 90.0 * DEGREE2RADIAN) * robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.123 );
        manipulator_link_data[6]->joint_axis            =   robotis_framework::getRotationY( 90.0 * DEGREE2RADIAN) * robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 1.0 );
        manipulator_link_data[6]->center_of_mass        =   robotis_framework::getRotationY( 90.0 * DEGREE2RADIAN) * robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
        manipulator_link_data[6]->joint_limit_max       =   M_PI;
        manipulator_link_data[6]->joint_limit_min       =   -M_PI;
        manipulator_link_data[6]->inertia               =   robotis_framework::getInertiaXYZ( 1.0 , 0.0 , 0.0 , 1.0 , 0.0 , 1.0 );

        manipulator_link_data[7]->name                  =   "grip";
        manipulator_link_data[7]->parent                =   6;
        manipulator_link_data[7]->sibling               =   -1;
        manipulator_link_data[7]->child                 =   8;
        manipulator_link_data[7]->mass                  =   0.509;
        manipulator_link_data[7]->relative_position     =   robotis_framework::getRotationY( 90.0 * DEGREE2RADIAN) * robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
        manipulator_link_data[7]->joint_axis            =   robotis_framework::getRotationY( 90.0 * DEGREE2RADIAN) * robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
        manipulator_link_data[7]->center_of_mass        =   robotis_framework::getRotationY( 90.0 * DEGREE2RADIAN) * robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
        manipulator_link_data[7]->joint_limit_max       =   100;
        manipulator_link_data[7]->joint_limit_min       =   -100.0;
        manipulator_link_data[7]->inertia               =   robotis_framework::getInertiaXYZ( 1.0 , 0.0 , 0.0 , 1.0 , 0.0 , 1.0 );

        manipulator_link_data[8]->name                  =   "end";
        manipulator_link_data[8]->parent                =   7;
        manipulator_link_data[8]->sibling               =   -1;
        manipulator_link_data[8]->child                 =   -1;
        manipulator_link_data[8]->mass                  =   0.0;
        manipulator_link_data[8]->relative_position     =   robotis_framework::getRotationY( 90.0 * DEGREE2RADIAN) * robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.12 );
        manipulator_link_data[8]->joint_axis            =   robotis_framework::getRotationY( 90.0 * DEGREE2RADIAN) * robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
        manipulator_link_data[8]->center_of_mass        =   robotis_framework::getRotationY( 90.0 * DEGREE2RADIAN) * robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
        manipulator_link_data[8]->joint_limit_max       =   100.0;
        manipulator_link_data[8]->joint_limit_min       =   -100.0;
        manipulator_link_data[8]->inertia               =   robotis_framework::getInertiaXYZ( 1.0 , 0.0 , 0.0 , 1.0 , 0.0 , 1.0 );
    }
}

std::vector<int> ManipulatorKinematicsDynamics::findRoute( int to )
{
    int _id = manipulator_link_data[ to ]->parent;

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

std::vector<int> ManipulatorKinematicsDynamics::findRoute( int from , int to )
{
    int _id = manipulator_link_data[ to ]->parent;

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

double ManipulatorKinematicsDynamics::totalMass( int joint_ID )
{
    double _mass;

    if ( joint_ID == -1 )
        _mass = 0.0;
    else
        _mass = manipulator_link_data[ joint_ID ]->mass + totalMass( manipulator_link_data[ joint_ID ]->sibling ) + totalMass( manipulator_link_data[ joint_ID ]->child );

    return _mass;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcMC( int joint_ID )
{
    Eigen::MatrixXd _mc(3,1);

    if ( joint_ID == -1 )
        _mc = Eigen::MatrixXd::Zero(3,1);
    else
    {
        _mc = manipulator_link_data[ joint_ID ]->mass * ( manipulator_link_data[ joint_ID ]->orientation * manipulator_link_data[ joint_ID ]->center_of_mass + manipulator_link_data[ joint_ID ]->position );
        _mc = _mc + calcMC( manipulator_link_data[ joint_ID ]->sibling ) + calcMC( manipulator_link_data[ joint_ID ]->child );
    }

    return _mc;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcCOM( Eigen::MatrixXd MC )
{
    double _mass ;
    Eigen::MatrixXd _COM( 3 , 1 );

    _mass = totalMass( 0 );

    _COM = MC / _mass;

    return _COM;
}

void ManipulatorKinematicsDynamics::forwardKinematics( int joint_ID )
{
    if ( joint_ID == -1 )
        return;

    manipulator_link_data[0]->position       = Eigen::MatrixXd::Zero(3,1);
    manipulator_link_data[0]->orientation    = robotis_framework::calcRodrigues( robotis_framework::calcHatto( manipulator_link_data[0]->joint_axis ), manipulator_link_data[ 0 ]->joint_angle );

    if ( joint_ID != 0 )
    {
        int _parent = manipulator_link_data[ joint_ID ]->parent;

        manipulator_link_data[ joint_ID ]->position =
                manipulator_link_data[ _parent ]->orientation * manipulator_link_data[ joint_ID ]->relative_position + manipulator_link_data[ _parent ]->position;
        manipulator_link_data[ joint_ID ]->orientation =
                manipulator_link_data[ _parent ]->orientation * robotis_framework::calcRodrigues( robotis_framework::calcHatto( manipulator_link_data[ joint_ID ]->joint_axis ), manipulator_link_data[ joint_ID ]->joint_angle );

        manipulator_link_data[ joint_ID ]->transformation.block<3, 1> ( 0 , 3) = manipulator_link_data[ joint_ID ]->position;
        manipulator_link_data[ joint_ID ]->transformation.block<3, 3> ( 0 , 0) = manipulator_link_data[ joint_ID ]->orientation;
    }

    forwardKinematics( manipulator_link_data[ joint_ID ]->sibling );
    forwardKinematics( manipulator_link_data[ joint_ID ]->child );
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcJacobian( std::vector<int> idx )
{
    int _idx_size 	= 	idx.size();
    int _end 		= 	_idx_size - 1;

    Eigen::MatrixXd _tar_position    = 	manipulator_link_data[ idx[ _end ] ]->position;
    Eigen::MatrixXd _Jacobian        = 	Eigen::MatrixXd::Zero( 6 , _idx_size );

    for ( int id = 0; id < _idx_size; id++)
    {
        int _id 					= 	idx[ id ];

        Eigen::MatrixXd _tar_orientation      = 	manipulator_link_data[ _id ]->orientation * manipulator_link_data[ _id ]->joint_axis;

        _Jacobian.block( 0 , id , 3 , 1 ) 	= 	robotis_framework::calcCross( _tar_orientation , _tar_position - manipulator_link_data[ _id ]->position );
        _Jacobian.block( 3 , id , 3 , 1 ) 	= 	_tar_orientation;
    }

    return _Jacobian;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcJacobianCOM(std::vector<int> idx)
{
    int _idx_size	=	idx.size();
    int _end 		=	_idx_size - 1;

    Eigen::MatrixXd _target_position    = 	manipulator_link_data[ idx[ _end ] ]->position;
    Eigen::MatrixXd _jacobianCOM        = 	Eigen::MatrixXd::Zero( 6 , _idx_size );

    for ( int id = 0; id < _idx_size; id++ )
    {
        int _id 			= 	idx[ id ];
        double _mass 	= 	totalMass( _id );

        Eigen::MatrixXd _og                     =	calcMC( _id ) / _mass - manipulator_link_data[ _id ]->position;
        Eigen::MatrixXd _target_orientation 	= 	manipulator_link_data[ _id ]->orientation * manipulator_link_data[ _id ]->joint_axis;

        _jacobianCOM.block( 0 , id , 3 , 1 ) 	= 	robotis_framework::calcCross( _target_orientation , _og );
        _jacobianCOM.block( 3 , id , 3 , 1 ) 	= 	_target_orientation;
    }

    return _jacobianCOM;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcVWerr( Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position, Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation )
{
    Eigen::MatrixXd _pos_err        =	tar_position - curr_position;
    Eigen::MatrixXd _ori_err        =	curr_orientation.inverse() * tar_orientation;
    Eigen::MatrixXd __ori_err      =	curr_orientation * robotis_framework::convertRotToOmega( _ori_err );

    Eigen::MatrixXd _err 	= 	Eigen::MatrixXd::Zero( 6 , 1 );
    _err.block(0,0,3,1) 		= 	_pos_err;
    _err.block(3,0,3,1) 		= 	__ori_err;

    return _err;
}

bool ManipulatorKinematicsDynamics::inverseKinematics( int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err )
{
    bool ik_success = false;
    bool limit_success = false;

    forwardKinematics(0);

    std::vector<int> _idx   = 	findRoute( to );

    for ( int iter = 0; iter < max_iter; iter++ )
    {
        Eigen::MatrixXd _Jacobian		= 	calcJacobian( _idx );

        Eigen::MatrixXd _curr_position      = 	manipulator_link_data[ to ]->position;
        Eigen::MatrixXd _curr_orientation 	= 	manipulator_link_data[ to ]->orientation;

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
            manipulator_link_data[ _joint_num ]->joint_angle    += 	_delta_angle.coeff( id );
        }

        forwardKinematics(0);
    }

    for ( int id = 0; id < _idx.size(); id++ )
    {
        int _joint_num      = 	_idx[ id ];

        if ( manipulator_link_data[ _joint_num ]->joint_angle >= manipulator_link_data[ _joint_num ]->joint_limit_max )
        {
            limit_success = false;
            break;
        }
        else if ( manipulator_link_data[ _joint_num ]->joint_angle <= manipulator_link_data[ _joint_num ]->joint_limit_min )
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

bool ManipulatorKinematicsDynamics::inverseKinematics( int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err )
{
    bool ik_success = false;
    bool limit_success = false;

    forwardKinematics(0);

    std::vector<int> _idx   = 	findRoute( from , to );

    for ( int iter = 0; iter < max_iter; iter++ )
    {
        Eigen::MatrixXd _Jacobian		= 	calcJacobian( _idx );

        Eigen::MatrixXd _curr_position      = 	manipulator_link_data[ to ]->position;
        Eigen::MatrixXd _curr_orientation 	= 	manipulator_link_data[ to ]->orientation;

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
            manipulator_link_data[ _joint_num ]->joint_angle    += 	_delta_angle.coeff( id );
        }

        forwardKinematics(0);
    }

    for ( int id = 0; id < _idx.size(); id++ )
    {
        int _joint_num      = 	_idx[ id ];

        if ( manipulator_link_data[ _joint_num ]->joint_angle >= manipulator_link_data[ _joint_num ]->joint_limit_max )
        {
            limit_success = false;
            break;
        }
        else if ( manipulator_link_data[ _joint_num ]->joint_angle <= manipulator_link_data[ _joint_num ]->joint_limit_min )
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
