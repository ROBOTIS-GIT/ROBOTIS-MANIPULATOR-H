#include "manipulator_demo_module/MoveitState.h"

namespace ROBOTIS_DEMO
{

MoveitState::MoveitState()
{
    points = 10;

    time_from_start = ros::Duration( 10.0 ); // movement duration

    display_planned_path_positions = Eigen::MatrixXd::Zero( points , MAX_JOINT_ID + 1 ); // positions of planned path
    display_planned_path_velocities = Eigen::MatrixXd::Zero( points , MAX_JOINT_ID + 1 ); // positions of planned path
    display_planned_path_accelerations = Eigen::MatrixXd::Zero( points , MAX_JOINT_ID + 1 ); // positions of planned path

    // initialization
    robot_model_loader.reset();
    planning_scene.reset();
}

MoveitState::~MoveitState(){}

void MoveitState::init( std::string description )
{
    // initialization
    robot_model_loader.reset( new robot_model_loader::RobotModelLoader( description ) );
    kinematic_model = robot_model_loader->getModel();
    kinematic_state.reset(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    arm_joint_model_group 	= 	kinematic_model->getJointModelGroup("arm");
    arm_joint_names          = 	arm_joint_model_group->getJointModelNames();

    planning_scene.reset( new planning_scene::PlanningScene( kinematic_model ) );
}

}


