/*
 * transformation.h
 *
 *  Created on: Sep 9, 2015
 *      Author: Changhyun Sung
 */

#ifndef DEMO_MODULE_TRANSFORMATION_H_
#define DEMO_MODULE_TRANSFORMATION_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <Eigen/Dense>

namespace ROBOTIS_DEMO
{

Eigen::MatrixXd transitionXYZ ( double position_x, double position_y, double position_z );
Eigen::MatrixXd transformationXYZRPY ( double position_x, double position_y, double position_z , double roll , double pitch , double yaw );
Eigen::MatrixXd inertiaXYZ( double ixx, double ixy, double ixz , double iyy , double iyz, double izz );

Eigen::MatrixXd rotationX( double angle );
Eigen::MatrixXd rotationY( double angle );
Eigen::MatrixXd rotationZ( double angle );

Eigen::MatrixXd rotation2rpy( Eigen::MatrixXd rotation );
Eigen::MatrixXd rpy2rotation( double roll, double pitch, double yaw );

Eigen::Quaterniond rpy2quaternion( double roll, double pitch, double yaw );
Eigen::Quaterniond rotation2quaternion( Eigen::MatrixXd rotation );

Eigen::MatrixXd quaternion2rpy( Eigen::Quaterniond quaternion );
Eigen::MatrixXd quaternion2rotation( Eigen::Quaterniond quaternion );

Eigen::MatrixXd rotation4d( double roll, double pitch, double yaw );

double sign( double x );
Eigen::MatrixXd hatto( Eigen::MatrixXd matrix3d );
Eigen::MatrixXd Rodrigues( Eigen::MatrixXd hat_matrix , double angle );
Eigen::MatrixXd rot2omega(Eigen::MatrixXd rotation );
Eigen::MatrixXd cross(Eigen::MatrixXd matrix3d_a, Eigen::MatrixXd matrix3d_b );

}

#endif /* DEMO_MODULE_TRANSFORMATION_H_ */
