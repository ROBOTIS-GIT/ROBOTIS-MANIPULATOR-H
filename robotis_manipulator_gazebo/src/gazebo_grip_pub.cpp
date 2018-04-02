/*
 * calc_test.cpp
 *
 *  Created on: Jul 11, 2015
 *      Author: sch
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>

ros::Publisher grip_joint_pub;

void grip_joint_callback( const std_msgs::Float64::ConstPtr& msg )
{
    std_msgs::Float64 _grip_joint_msg;

    _grip_joint_msg.data = msg->data;

    grip_joint_pub.publish( _grip_joint_msg );
}

int main( int argc , char **argv )
{
    ros::init( argc , argv , "gazebo_gripper_publisher" );
    ros::NodeHandle nh("~");

    grip_joint_pub  = nh.advertise<std_msgs::Float64>("/robotis_manipulator/grip_1_pos/command", 0);

    ros::Subscriber grip_joint_sub = nh.subscribe("/robotis_manipulator/grip_pos/command", 5, grip_joint_callback);

	ros::spin();

    return 0;
}
