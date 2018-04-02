/*
 * trajectory.h
 *
 *  Created on: Jul 13, 2015
 *      Author: sch
 */

#ifndef DEMO_MODULE_TRAJECTORY_H_
#define DEMO_MODULE_TRAJECTORY_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <Eigen/Dense>

// minimum jerk trajectory

namespace ROBOTIS_DEMO
{

Eigen::MatrixXd minimum_jerk_tra( double pos_start , double vel_start , double accel_start,
                                  double pos_end ,   double vel_end ,   double accel_end,
                                  double smp_time ,  double mov_time );

Eigen::MatrixXd minimum_jerk_tra_vian_qdqddq( int via_num,
                                              double pos_start , double vel_start , double accel_start ,
                                              Eigen::MatrixXd pos_via,  Eigen::MatrixXd vel_via, Eigen::MatrixXd accel_via,
                                              double pos_end, double vel_end, double accel_end,
                                              double smp_time, Eigen::MatrixXd via_time, double mov_time );

Eigen::MatrixXd arc3d_tra( double smp_time, double mov_time,
                           Eigen::MatrixXd center_point, Eigen::MatrixXd normal_vector, Eigen::MatrixXd start_point,
                           double rotation_angle, double cross_ratio );

}

#endif /* DEMO_MODULE_TRAJECTORY_H_ */
