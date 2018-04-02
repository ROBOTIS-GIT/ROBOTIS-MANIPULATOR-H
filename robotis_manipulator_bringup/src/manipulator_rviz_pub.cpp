/*
 * calc_test.cpp
 *
 *  Created on: Jul 11, 2015
 *      Author: sch
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

ros::Publisher present_joint_states_pub;
ros::Publisher goal_joint_states_pub;

void present_joint_states_callback( const sensor_msgs::JointState::ConstPtr& msg )
{
    sensor_msgs::JointState _present_msg;

    for ( int _index = 0 ; _index < msg->name.size(); _index++ )
    {
        _present_msg.name.push_back( msg->name[ _index ] );
        _present_msg.position.push_back( msg->position[ _index ] );

        if ( _present_msg.name[ _index ] == "grip" )
        {
            _present_msg.name.push_back("grip_1");
            _present_msg.position.push_back(_present_msg.position[ _index ]);
        }
    }
    present_joint_states_pub.publish( _present_msg );
}

void goal_joint_states_callback( const sensor_msgs::JointState::ConstPtr& msg )
{
    sensor_msgs::JointState _goal_msg;

    for ( int _index = 0 ; _index < msg->name.size(); _index++ )
    {
        _goal_msg.name.push_back( msg->name[ _index ] );
        _goal_msg.position.push_back( msg->position[ _index ] );

        if ( _goal_msg.name[ _index ] == "grip" )
        {
            _goal_msg.name.push_back("grip_1");
            _goal_msg.position.push_back(_goal_msg.position[ _index ]);
        }
    }
    goal_joint_states_pub.publish( _goal_msg );
}

int main( int argc , char **argv )
{
    ros::init( argc , argv , "manipulation_rviz_publisher" );
    ros::NodeHandle nh("~");

    present_joint_states_pub  = nh.advertise<sensor_msgs::JointState>("/robotis/manipulator/present_joint_states", 0);
    goal_joint_states_pub  = nh.advertise<sensor_msgs::JointState>("/robotis/manipulator/goal_joint_states", 0);

    ros::Subscriber present_joint_states_sub = nh.subscribe("/robotis/present_joint_states", 5, present_joint_states_callback);
    ros::Subscriber goal_joint_states_sub = nh.subscribe("/robotis/goal_joint_states", 5, goal_joint_states_callback);

	ros::spin();

    return 0;
}
