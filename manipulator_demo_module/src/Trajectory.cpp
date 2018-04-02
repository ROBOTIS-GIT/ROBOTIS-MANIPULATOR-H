/*
 * trajectory.cpp
 *
 *  Created on: Jul 13, 2015
 *      Author: sch
 */

#include <math.h>

#include "manipulator_demo_module/Trajectory.h"

namespace ROBOTIS_DEMO
{

Eigen::MatrixXd minimum_jerk_tra( double pos_start , double vel_start , double accel_start,
                                  double pos_end ,   double vel_end ,   double accel_end,
                                  double smp_time ,  double mov_time )
/*
   simple minimum jerk trajectory

   pos_start : position at initial state
   vel_start : velocity at initial state
   accel_start : acceleration at initial state

   pos_end : position at final state
   vel_end : velocity at final state
   accel_end : acceleration at final state

   smp_time : sampling time

   mov_time : movement time
*/

{
    Eigen::MatrixXd ___C( 3 , 3 );
    Eigen::MatrixXd __C( 3 , 1 );

    ___C << pow( mov_time , 3 )	   , pow( mov_time , 4 )	  , pow( mov_time , 5 ),
           3 * pow( mov_time , 2 ) , 4 * pow( mov_time , 3 )  , 5 * pow( mov_time , 4 ),
           6 * mov_time		       , 12 * pow( mov_time , 2 ) , 20 * pow( mov_time , 3 );

    __C << pos_end - pos_start - vel_start * mov_time - accel_start * pow( mov_time , 2 ) / 2,
          vel_end - vel_start - accel_start * mov_time,
          accel_end - accel_start ;

    Eigen::Matrix<double,3,1> _C = ___C.inverse() * __C;

    double _time_steps;

    _time_steps = mov_time / smp_time;
    int __time_steps = round( _time_steps + 1 );

    Eigen::MatrixXd _time = Eigen::MatrixXd::Zero( __time_steps , 1 );
    Eigen::MatrixXd _tra = Eigen::MatrixXd::Zero( __time_steps , 1 );

    for ( int step = 0; step < __time_steps; step++ )
        _time.coeffRef( step , 0 ) = step * smp_time;

    for ( int step = 0; step < __time_steps; step++ )
	{
        _tra.coeffRef( step , 0 ) =
                pos_start +
                vel_start * _time.coeff( step , 0 ) +
                0.5 * accel_start * pow( _time.coeff( step , 0 ) , 2 ) +
                _C.coeff( 0 , 0 ) * pow( _time.coeff( step , 0 ) , 3 ) +
                _C.coeff( 1 , 0 ) * pow( _time.coeff( step , 0 ) , 4 ) +
                _C.coeff( 2 , 0 ) * pow( _time.coeff( step , 0 ) , 5 );
	}

    return _tra;
}

Eigen::MatrixXd minimum_jerk_tra_vian_qdqddq( int via_num,
                                              double pos_start , double vel_start , double accel_start ,
                                              Eigen::MatrixXd pos_via,  Eigen::MatrixXd vel_via, Eigen::MatrixXd accel_via,
                                              double pos_end, double vel_end, double accel_end,
                                              double smp_time, Eigen::MatrixXd via_time, double mov_time )
/*
   minimum jerk trajectory with via-points
   (via-point constraints: position and velocity at each point)

   n  : the number of via-points

   x0 : position at initial state
   v0 : velocity at initial state
   a0 : acceleration at initial state

   x  : position matrix at via-points state ( size : n x 1 )
   dx : velocity matrix at via-points state ( size : n x 1 )
   ddx : acceleration matrix at via-points state ( size : n x 1 )

   xf : position at final state
   vf : velocity at final state
   af : acceleration at final state

   smp : sampling time

   t  : time matrix passing through via-points state ( size : n x 1 )
   tf : movement time
*/

{
    int i,j,k ;

    /* Calculation Matrix B */

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero( 3 * via_num + 3 , 1 );

    for (i=1; i<=via_num; i++){
        B.coeffRef(3*i-3,0) =
                pos_via.coeff(i-1,0) -
                pos_start -
                vel_start*via_time.coeff(i-1,0) -
                (accel_start/2)*pow(via_time.coeff(i-1,0),2) ;

        B.coeffRef(3*i-2,0) =
                vel_via.coeff(i-1,0) -
                vel_start -
                accel_start*via_time.coeff(i-1,0) ;

        B.coeffRef(3*i-1,0) =
                accel_via.coeff(i-1,0) -
                accel_start ;
    }

    B.coeffRef(3*via_num,0) =
            pos_end -
            pos_start -
            vel_start*mov_time -
            (accel_start/2)*pow(mov_time,2) ;

    B.coeffRef(3*via_num+1,0) =
            vel_end -
            vel_start -
            accel_start*mov_time ;

    B.coeffRef(3*via_num+2,0) =
            accel_end -
            accel_start ;


    /* Calculation Matrix A */

    Eigen::MatrixXd A1 = Eigen::MatrixXd::Zero(3*via_num,3);

    for (i=1; i<=via_num; i++){
        A1.coeffRef(3*i-3,0) = pow(via_time.coeff(i-1,0),3) ;
        A1.coeffRef(3*i-3,1) = pow(via_time.coeff(i-1,0),4) ;
        A1.coeffRef(3*i-3,2) = pow(via_time.coeff(i-1,0),5) ;

        A1.coeffRef(3*i-2,0) = 3*pow(via_time.coeff(i-1,0),2) ;
        A1.coeffRef(3*i-2,1) = 4*pow(via_time.coeff(i-1,0),3) ;
        A1.coeffRef(3*i-2,2) = 5*pow(via_time.coeff(i-1,0),4) ;

        A1.coeffRef(3*i-1,0) = 6*via_time.coeff(i-1,0) ;
        A1.coeffRef(3*i-1,1) = 12*pow(via_time.coeff(i-1,0),2) ;
        A1.coeffRef(3*i-1,2) = 20*pow(via_time.coeff(i-1,0),3) ;
    }


    Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(3*via_num,3*via_num);

    for (i=1; i<=via_num; i++){
        for (j=1; j<=via_num; j++){
            if (i > j){
                k = i ;
            }else{
                k = j ;
            }
            A2.coeffRef(3*j-3,3*i-3) = pow((via_time.coeff(k-1,0)-via_time.coeff(i-1,0)),3)/6 ;
            A2.coeffRef(3*j-3,3*i-2) = pow((via_time.coeff(k-1,0)-via_time.coeff(i-1,0)),4)/24 ;
            A2.coeffRef(3*j-3,3*i-1) = pow((via_time.coeff(k-1,0)-via_time.coeff(i-1,0)),5)/120 ;

            A2.coeffRef(3*j-2,3*i-3) = pow((via_time.coeff(k-1,0)-via_time.coeff(i-1,0)),2)/2 ;
            A2.coeffRef(3*j-2,3*i-2) = pow((via_time.coeff(k-1,0)-via_time.coeff(i-1,0)),3)/6 ;
            A2.coeffRef(3*j-2,3*i-1) = pow((via_time.coeff(k-1,0)-via_time.coeff(i-1,0)),4)/24 ;

            A2.coeffRef(3*j-1,3*i-3) = via_time.coeff(k-1,0)-via_time.coeff(i-1,0) ;
            A2.coeffRef(3*j-1,3*i-2) = pow((via_time.coeff(k-1,0)-via_time.coeff(i-1,0)),2)/2 ;
            A2.coeffRef(3*j-1,3*i-1) = pow((via_time.coeff(k-1,0)-via_time.coeff(i-1,0)),3)/6 ;
        }
    }


    Eigen::MatrixXd A3 = Eigen::MatrixXd::Zero(3,3*via_num+3);

    A3.coeffRef(0,0) = pow(mov_time,3);
    A3.coeffRef(0,1) = pow(mov_time,4);
    A3.coeffRef(0,2) = pow(mov_time,5);

    A3.coeffRef(1,0) = 3*pow(mov_time,2);
    A3.coeffRef(1,1) = 4*pow(mov_time,3);
    A3.coeffRef(1,2) = 5*pow(mov_time,4);

    A3.coeffRef(2,0) = 6*mov_time;
    A3.coeffRef(2,1) = 12*pow(mov_time,2);
    A3.coeffRef(2,2) = 20*pow(mov_time,3);

    for (i=1; i<=via_num; i++){
        A3.coeffRef(0,3*i) = pow(mov_time-via_time.coeff(i-1,0),3)/6 ;
        A3.coeffRef(1,3*i) = pow(mov_time-via_time.coeff(i-1,0),2)/2 ;
        A3.coeffRef(2,3*i) = mov_time-via_time.coeff(i-1,0) ;

        A3.coeffRef(0,3*i+1) = pow(mov_time-via_time.coeff(i-1,0),4)/24 ;
        A3.coeffRef(1,3*i+1) = pow(mov_time-via_time.coeff(i-1,0),3)/6 ;
        A3.coeffRef(2,3*i+1) = pow(mov_time-via_time.coeff(i-1,0),2)/2 ;

        A3.coeffRef(0,3*i+2) = pow(mov_time-via_time.coeff(i-1,0),5)/120 ;
        A3.coeffRef(1,3*i+2) = pow(mov_time-via_time.coeff(i-1,0),4)/24 ;
        A3.coeffRef(2,3*i+2) = pow(mov_time-via_time.coeff(i-1,0),3)/6 ;
    }

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3*via_num+3,3*via_num+3);

    A.block(0,0,3*via_num,3) = A1 ;
    A.block(0,3,3*via_num,3*via_num) = A2 ;
    A.block(3*via_num,0,3,3*via_num+3) = A3 ;

    /* Calculation Matrix C (coefficient of polynomial function) */

    Eigen::MatrixXd C(3*via_num+3,1);
    //C = A.inverse()*B;
    C = A.colPivHouseholderQr().solve(B);

    /* Time */

    int NN;
    double N;

    N = mov_time/smp_time ;
    NN = round(N) ;

    Eigen::MatrixXd Time = Eigen::MatrixXd::Zero(NN+1,1);

    for (i=1; i<=NN+1; i++){
        Time.coeffRef(i-1,0) = (i-1)*smp_time;
    }

    /* Time_via */

    Eigen::MatrixXd Time_via = Eigen::MatrixXd::Zero(via_num,1);

    for (i=1; i<=via_num; i++){
        Time_via.coeffRef(i-1,0) = round(via_time.coeff(i-1,0)/smp_time)+2;
    }

    /* Minimum Jerk Trajectory with Via-points */

    Eigen::MatrixXd _tra_jerk_via = Eigen::MatrixXd::Zero(NN+1,1);

    for (i=1; i<=NN+1; i++){
        _tra_jerk_via.coeffRef(i-1,0) =
                pos_start +
                vel_start*Time.coeff(i-1,0) +
                0.5*accel_start*pow(Time.coeff(i-1,0),2) +
                C.coeff(0,0)*pow(Time.coeff(i-1,0),3) +
                C.coeff(1,0)*pow(Time.coeff(i-1,0),4) +
                C.coeff(2,0)*pow(Time.coeff(i-1,0),5) ;
    }

    for (i=1; i<=via_num; i++){
        for (j=Time_via.coeff(i-1,0); j<=NN+1; j++){
            _tra_jerk_via.coeffRef(j-1,0) =
                    _tra_jerk_via.coeff(j-1,0) +
                    C.coeff(3*i,0)*pow((Time.coeff(j-1,0)-via_time.coeff(i-1,0)),3)/6 +
                    C.coeff(3*i+1,0)*pow((Time.coeff(j-1,0)-via_time.coeff(i-1,0)),4)/24 +
                    C.coeff(3*i+2,0)*pow((Time.coeff(j-1,0)-via_time.coeff(i-1,0)),5)/120 ;

        }
    }

    return _tra_jerk_via;

}

Eigen::MatrixXd arc3d_tra( double smp_time, double mov_time,
                           Eigen::MatrixXd center_point, Eigen::MatrixXd normal_vector, Eigen::MatrixXd start_point,
                           double rotation_angle, double cross_ratio )
{
    int _all_time_steps = int ( round( mov_time / smp_time ) ) + 1 ;

    Eigen::MatrixXd _angle_tra = minimum_jerk_tra( 0.0 , 0.0 , 0.0 ,
                                                   rotation_angle , 0.0 , 0.0 ,
                                                   smp_time , mov_time );

    Eigen::MatrixXd _pt = Eigen::MatrixXd::Zero( 3 , _all_time_steps );

    for (int i = 0; i < _all_time_steps; i++ )
    {
        double _t = ( ( double ) i ) * smp_time ;

        double _th = _angle_tra.coeff( i , 0 );//( _t / mov_time ) * rotation_angle;

        Eigen::MatrixXd _w_wedge( 3 , 3 );

        _w_wedge <<  0.0		             , -normal_vector.coeff(2,0),  normal_vector.coeff(1,0),
                     normal_vector.coeff(2,0),  0.0                     , -normal_vector.coeff(0,0),
                    -normal_vector.coeff(1,0),  normal_vector.coeff(0,0),  0.0                     ;

        Eigen::MatrixXd _E = Eigen::MatrixXd::Identity( 3 , 3 );

        Eigen::MatrixXd _R = _E + _w_wedge * sin( _th ) + _w_wedge * _w_wedge * ( 1 - cos( _th ) );

        double _cross = cross_ratio * ( 1.0 - 2.0 * ( abs ( 0.5 - _t/mov_time ) ) );

        _pt.block( 0 , i , 3 , 1 ) = ( 1 + _cross ) * ( _R * ( start_point - center_point ) ) + center_point;
    }

    Eigen::MatrixXd _pt_trans = _pt.transpose();

    return _pt_trans;
}


}
