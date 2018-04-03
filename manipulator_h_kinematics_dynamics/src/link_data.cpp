/*
 * Link.cpp
 *
 *  Created on: Jan 11, 2016
 *      Author: sch
 */

#include "manipulator_h_kinematics_dynamics/link_data.h"

namespace ROBOTIS
{

LinkData::LinkData()
{
    name = "";

    parent = -1;
    sibling = -1;
    child = -1;

    mass = 0.0;

    relative_position = robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    joint_axis = robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    center_of_mass = robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    inertia = robotis_framework::getInertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

    joint_limit_max = 100.0;
    joint_limit_min = -100.0;

    joint_angle = 0.0;
    joint_velocity = 0.0;
    joint_acceleration = 0.0;

    position = robotis_framework::getTransitionXYZ( 0.0 , 0.0 , 0.0 );
    orientation = robotis_framework::convertRPYToRotation( 0.0 , 0.0 , 0.0 );
    transformation = robotis_framework::getTransformationXYZRPY( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0);
}

LinkData::~LinkData(){}

}
