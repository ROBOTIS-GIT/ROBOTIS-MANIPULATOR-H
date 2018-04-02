/*
 * jointstate.cpp
 *
 *  Created on: Jul 10, 2015
 *      Author: sch
 */

#include "manipulator_demo_module/RobotisCommon.h"
#include "manipulator_demo_module/JointState.h"

namespace ROBOTIS_DEMO
{

JointData JointState::goal_joint_state[ MAX_JOINT_ID + 1 ]; // goal state
JointData JointState::curr_joint_state[ MAX_JOINT_ID + 1 ]; // current state
JointData JointState::fake_joint_state[ MAX_JOINT_ID + 1 ]; // current state

}

