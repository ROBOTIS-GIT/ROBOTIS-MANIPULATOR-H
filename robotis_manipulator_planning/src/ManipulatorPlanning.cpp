
#include <ros/ros.h>

#include <pthread.h>

#include <std_msgs/String.h>

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

#include "manipulator_demo_module_msgs/KinematicsPose.h"

//ros::Publisher current_text_pub;
ros::Publisher execute_planned_path_msg_pub;
pthread_t tra_gene;
manipulator_demo_module_msgs::KinematicsPose kinematics_pose_msg;

void* plan_trajectory_proc(void* arg)
{
//    std_msgs::String text_msg;
//    text_msg.data = "Planning...";
//    current_text_pub.publish( text_msg );

    static moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::MoveGroup::Plan my_plan;

    bool success;

    /* ----- set planning time ----- */

    group.setPlanningTime(5.0);

    /* set start state */
    group.setStartState(*group.getCurrentState());

    /* set target state */
    geometry_msgs::Pose target_pose;

    target_pose.position.x = kinematics_pose_msg.pose.position.x;
    target_pose.position.y = kinematics_pose_msg.pose.position.y;
    target_pose.position.z = kinematics_pose_msg.pose.position.z;

    target_pose.orientation.x = kinematics_pose_msg.pose.orientation.x;
    target_pose.orientation.y = kinematics_pose_msg.pose.orientation.y;
    target_pose.orientation.z = kinematics_pose_msg.pose.orientation.z;
    target_pose.orientation.w = kinematics_pose_msg.pose.orientation.w;

    group.setPoseTarget( target_pose );

    /* motion planning */
    success = group.plan( my_plan );

    if ( success == true )
    {
        ROS_INFO("Planning Success");

        std_msgs::String msg;
        msg.data = "execute";

        execute_planned_path_msg_pub.publish( msg );
    }
    else
    {
        ROS_INFO("Planning Fail");

        std_msgs::String msg;
        msg.data = "fail";

        execute_planned_path_msg_pub.publish( msg );
    }
}

void kinematics_pose_msg_callback( const manipulator_demo_module_msgs::KinematicsPose::ConstPtr& msg )
{
    kinematics_pose_msg = *msg;

    pthread_create( &tra_gene , NULL , plan_trajectory_proc , NULL );

    return;
}

int main( int argc , char **argv )
{
    ros::init( argc , argv , "manipulator_planning_publisher" );
    ros::NodeHandle nh("~");

    execute_planned_path_msg_pub = nh.advertise<std_msgs::String>("/robotis/demo/execute_planned_path", 0);
//    current_text_pub = nh.advertise<std_msgs::String>("/robotis/current_text", 0);

    ros::Subscriber kinematics_pose_msg_sub = nh.subscribe("/robotis/demo/kinematics_pose_msg", 5, kinematics_pose_msg_callback);

    ros::spin();

    return 0;
}
