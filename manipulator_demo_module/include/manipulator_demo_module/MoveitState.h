#ifndef DEMO_MODULE_MOVEITSTATE_H_
#define DEMO_MODULE_MOVEITSTATE_H_

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointControllerState.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include "robotis_math/robotis_math.h"
#include "manipulator_kinematics_dynamics/ManipulatorKinematicsDynamics.h"

namespace ROBOTIS_DEMO
{

class MoveitState
{
public:

    MoveitState();
    ~MoveitState();

    int points; // planned number of via-points

    ros::Duration time_from_start; // planned movement time

    Eigen::MatrixXd display_planned_path_positions; // planned position trajectory
    Eigen::MatrixXd display_planned_path_velocities; // planned velocity trajectory
    Eigen::MatrixXd display_planned_path_accelerations; // planned acceleration trajectory

    moveit_msgs::DisplayTrajectory moveit_msg;

    // initialization
    boost::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader;
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;

    robot_state::JointModelGroup* 	arm_joint_model_group;
    std::vector<std::string> 		arm_joint_names;

    std::vector<double> _arm_joint_values , arm_joint_values ;

    boost::shared_ptr<planning_scene::PlanningScene>   planning_scene;

    // moveit path planning
    boost::shared_ptr<moveit::planning_interface::MoveGroup> arm_planning_group;

    void init( std::string description );

};

}

#endif /* DEMO_MODULE_MOVEITSTATE_H_ */
