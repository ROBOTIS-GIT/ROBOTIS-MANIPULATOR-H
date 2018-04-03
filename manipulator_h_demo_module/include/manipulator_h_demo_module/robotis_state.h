
#ifndef DEMO_MODULE_ROBOTISSTATE_H_
#define DEMO_MODULE_ROBOTISSTATE_H_

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include "robotis_math/robotis_math.h"
#include "manipulator_h_kinematics_dynamics/manipulator_h_kinematics_dynamics.h"

#include "manipulator_h_demo_module_msgs/JointPose.h"
#include "manipulator_h_demo_module_msgs/KinematicsPose.h"

namespace ROBOTIS_DEMO
{

class RobotisState
{
public:

    RobotisState();
    ~RobotisState();

    bool is_moving;

	int cnt; // counter number

	double mov_time; // movement time
	double smp_time; // sampling time

    int all_time_steps; // all time steps of movement time

    Eigen::MatrixXd calc_joint_tra; // calculated joint trajectory
    Eigen::MatrixXd calc_task_tra; // calculated task trajectory

    Eigen::MatrixXd joint_ini_pose;

    int via_num;
    Eigen::MatrixXd via_time;

    // for gui
    manipulator_h_demo_module_msgs::JointPose joint_pose_msg;
    manipulator_h_demo_module_msgs::KinematicsPose kinematics_pose_msg;

    // inverse kinematics
    bool ik_solve;
    Eigen::MatrixXd ik_target_position;
    Eigen::MatrixXd ik_start_rotation , ik_target_rotation;
    int ik_id_start;
    int ik_id_end;

    // msgs
    std_msgs::String send_tra_msg;

    void setInverseKinematics(int cnt , Eigen::MatrixXd start_rotation);

    //

    Eigen::MatrixXd curr_position;
    Eigen::MatrixXd curr_orientation;

    bool execute_planned_path ;


};

}

#endif /* DEMO_MODULE_ROBOTISSTATE_H_ */
