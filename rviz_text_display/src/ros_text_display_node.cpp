
#include <stdio.h>
#include "rviz_text_display/ros_text_display_node.h"

#include <cmath>

namespace rviz_robotis
{

RosTextDisplayNode::RosTextDisplayNode()
//    : marker_name_("pose_marker")
{
    init();
}

RosTextDisplayNode::~RosTextDisplayNode()
{
    /*
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
    */
}

bool RosTextDisplayNode::init()
{    
    // subscriber, publisher
//    pose_pub_ = nh_.advertise<geometry_msgs::Pose>("/pose_panel/pose", 1 );
//    point_pub_ = nh_.advertise<geometry_msgs::Point>("/pose_panel/point", 1 );
//    clicked_point_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("clicked_point", 0, &RosTextDisplayNode::pointStampedCallback, this);
//    interactive_marker_server_.reset( new interactive_markers::InteractiveMarkerServer("Pose","",false) );

    current_text_sub_ = nh_.subscribe<std_msgs::String>("/robotis/current_text", 0, &RosTextDisplayNode::CurrentTextCallback, this);

    // start QThread
    start();

    return true;
}

// thread run
void RosTextDisplayNode::run() {

    // setting rate : 1 Hz
    ros::Rate loop_rate(100);

    while ( ros::ok() ) {

        // ...

        ros::spinOnce();
        loop_rate.sleep();
    }

//    interactive_marker_server_.reset();

    std::cout << "Panel shutdown." << std::endl;

    // Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void RosTextDisplayNode::CurrentTextCallback( const std_msgs::String::ConstPtr & msg )
{
    Q_EMIT updateCurrentText( tr(msg->data.c_str()) );
}


//void RosTextDisplayNode::pointStampedCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
//{
//    ROS_INFO("clicked_point");

//    frame_id_ = msg->header.frame_id;

//    // update point ui
//    Q_EMIT updateCurrentPoint(msg->point);
//}

//void RosTextDisplayNode::interactiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
//{
//    // event
//    switch ( feedback->event_type )
//    {
//        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
//            break;

//        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
//            break;

//        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
//        {
//            current_pose_ = feedback->pose;

//            // update pose ui
//            Q_EMIT updateCurrentPose(feedback->pose);

//            break;
//        }
//        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
//            break;

//        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
//            break;
//    }

//    interactive_marker_server_->applyChanges();
//}

//void RosTextDisplayNode::makeInteractiveMarker(const geometry_msgs::Pose &pose)
//{
//    if(frame_id_ == "")
//    {
//        ROS_ERROR("No frame id!!!");
//        return;
//    }

//    ROS_INFO_STREAM("Make Interactive Marker! - " << pose.position.x << ", " << pose.position.y << ", " << pose.position.z
//                    << " [" <<  pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << " | " << pose.orientation.w << "]");

//    interactive_marker_server_->clear();

//    visualization_msgs::InteractiveMarker _interactive_marker;
//    _interactive_marker.pose = pose;    // set pose


//    // Visualize Interactive Marker
//    _interactive_marker.header.frame_id = frame_id_;
//    _interactive_marker.scale = 0.3;

//    _interactive_marker.name = "pose_marker";
//    _interactive_marker.description = "3D Pose Control";

//    // ----- center marker
//    visualization_msgs::InteractiveMarkerControl _interactive_marker_control;

//    _interactive_marker_control.always_visible = true;

//    visualization_msgs::Marker _marker;

//    _marker.type = visualization_msgs::Marker::CUBE;

//    // center cube
//    _marker.scale.x = 0.03;
//    _marker.scale.y = 0.03;
//    _marker.scale.z = 0.03;

//    _marker.color.r = 1.0;
//    _marker.color.g = 0.5;
//    _marker.color.b = 0.5;
//    _marker.color.a = 1.0;

//    _interactive_marker_control.markers.push_back(_marker);

//    // axis x
//    _marker.pose.position.x = 0.05;
//    _marker.pose.position.y = 0.0;
//    _marker.pose.position.z = 0.0;

//    _marker.scale.x = 0.1;
//    _marker.scale.y = 0.01;
//    _marker.scale.z = 0.01;

//    _marker.color.r = 1.0;
//    _marker.color.g = 0.0;
//    _marker.color.b = 0.0;
//    _marker.color.a = 1.0;

//    _interactive_marker_control.markers.push_back(_marker);

//    // axis y
//    _marker.pose.position.x = 0.0;
//    _marker.pose.position.y = 0.05;
//    _marker.pose.position.z = 0.0;

//    _marker.scale.x = 0.01;
//    _marker.scale.y = 0.1;
//    _marker.scale.z = 0.01;

//    _marker.color.r = 0.0;
//    _marker.color.g = 1.0;
//    _marker.color.b = 0.0;
//    _marker.color.a = 1.0;

//    _interactive_marker_control.markers.push_back(_marker);

//    // axis z
//    _marker.pose.position.x = 0.0;
//    _marker.pose.position.y = 0.0;
//    _marker.pose.position.z = 0.05;

//    _marker.scale.x = 0.01;
//    _marker.scale.y = 0.01;
//    _marker.scale.z = 0.1;

//    _marker.color.r = 0.0;
//    _marker.color.g = 0.0;
//    _marker.color.b = 1.0;
//    _marker.color.a = 1.0;

//    _interactive_marker_control.markers.push_back(_marker);

//    _interactive_marker.controls.push_back(_interactive_marker_control);

//    // ----- controller
//    visualization_msgs::InteractiveMarkerControl _interactive_control;
//    _interactive_marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;

//    // move and rotate along axis x : default
//    _interactive_control.orientation.x = 1;
//    _interactive_control.orientation.y = 0;
//    _interactive_control.orientation.z = 0;
//    _interactive_control.orientation.w = 1;
//    _interactive_control.name = "rotate";
//    _interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
//    _interactive_marker.controls.push_back(_interactive_control);
//    _interactive_control.name = "move";
//    _interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
//    _interactive_marker.controls.push_back(_interactive_control);

//    // move and rotate along axis y
//    _interactive_control.orientation.x = 0;
//    _interactive_control.orientation.y = 1;
//    _interactive_control.orientation.z = 0;
//    _interactive_control.orientation.w = 1;
//    _interactive_control.name = "rotate";
//    _interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
//    _interactive_marker.controls.push_back(_interactive_control);
//    _interactive_control.name = "move";
//    _interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
//    _interactive_marker.controls.push_back(_interactive_control);

//    // move and rotate along axis z
//    _interactive_control.orientation.x = 0;
//    _interactive_control.orientation.y = 0;
//    _interactive_control.orientation.z = 1;
//    _interactive_control.orientation.w = 1;
//    _interactive_control.name = "rotate";
//    _interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
//    _interactive_marker.controls.push_back(_interactive_control);
//    _interactive_control.name = "move";
//    _interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
//    _interactive_marker.controls.push_back(_interactive_control);

//    interactive_marker_server_->insert(_interactive_marker);
//    interactive_marker_server_->setCallback(_interactive_marker.name, boost::bind(&RosTextDisplayNode::interactiveMarkerFeedback, this, _1));

//    interactive_marker_server_->applyChanges();
//}

//void RosTextDisplayNode::updateInteractiveMarker(const geometry_msgs::Pose &pose)
//{
//    ROS_INFO("Update Interactive Marker Pose");

//    visualization_msgs::InteractiveMarker _interactive_marker;
//    if(!(interactive_marker_server_->get(marker_name_, _interactive_marker)))
//    {
//        ROS_ERROR("No Interactive marker to set pose");
//        return;
//    }

//    interactive_marker_server_->setPose(_interactive_marker.name, pose);
//    interactive_marker_server_->applyChanges();
//}

//void RosTextDisplayNode::updateCurrentText( const std_msgs::String &msg )
//{
//    ROS_INFO("1");

//}

//void RosTextDisplayNode::getInteractiveMarkerPose()
//{
//    ROS_INFO("Get Interactive Marker Pose");

//    visualization_msgs::InteractiveMarker _interactive_marker;
//    if(!(interactive_marker_server_->get(marker_name_, _interactive_marker)))
//    {
//        ROS_ERROR("No Interactive marker to get pose");
//        return;
//    }

//    // update pose ui
//    Q_EMIT updateCurrentPose(_interactive_marker.pose);

//    clearInteractiveMarker();
//}

//void RosTextDisplayNode::clearInteractiveMarker()
//{
//    ROS_INFO("Clear Interactive Marker");

//    // clear and apply
//    interactive_marker_server_->clear();
//    interactive_marker_server_->applyChanges();
//}

// publish 3d point
//void RosTextDisplayNode::publishPoint(const geometry_msgs::Point &point)
//{
//    point_pub_.publish(point);
//    ROS_INFO_STREAM("Publish Point : " << point.x << ", " << point.y << ", " << point.z);
//}

// publish 3d pose
//void RosTextDisplayNode::publishPose(const geometry_msgs::Pose &pose)
//{
//    pose_pub_.publish(pose);
//    ROS_INFO_STREAM("Publish Pose : " << pose.position.x << ", " << pose.position.y << ", " << pose.position.z
//                    << " [" <<  pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << " | " << pose.orientation.w << "]");
//}

// math : euler & quaternion & rotation mat
//Eigen::Vector3d RosTextDisplayNode::rotation2rpy(const Eigen::MatrixXd &rotation )
//{
//    Eigen::Vector3d _rpy;

//    _rpy[0] = atan2( rotation.coeff( 2 , 1 ), rotation.coeff( 2 , 2 ) );
//    _rpy[1] = atan2(-rotation.coeff( 2 , 0 ), sqrt( pow( rotation.coeff( 2 , 1 ) , 2 ) + pow( rotation.coeff( 2 , 2 ) , 2 ) ) );
//    _rpy[2] = atan2( rotation.coeff( 1 , 0 ), rotation.coeff( 0 , 0 ) );

//    return _rpy;
//}

//Eigen::MatrixXd RosTextDisplayNode::rpy2rotation(const double &roll, const double &pitch, const double &yaw )
//{
//    Eigen::MatrixXd _rotation = rotationZ( yaw ) * rotationY( pitch ) * rotationX( roll );

//    return _rotation;
//}

//Eigen::Quaterniond RosTextDisplayNode::rpy2quaternion(const Eigen::Vector3d &euler)
//{
//    return rpy2quaternion(euler[0], euler[1], euler[2]);
//}

//Eigen::Quaterniond RosTextDisplayNode::rpy2quaternion(const double &roll, const double &pitch, const double &yaw )
//{
//    Eigen::MatrixXd _rotation = rpy2rotation( roll, pitch, yaw );

//    Eigen::Matrix3d _rotation3d;
//    _rotation3d = _rotation.block( 0 , 0 , 3 , 3 );

//    Eigen::Quaterniond _quaternion;

//    _quaternion = _rotation3d;

//    return _quaternion;
//}

//Eigen::Quaterniond RosTextDisplayNode::rotation2quaternion(const Eigen::MatrixXd &rotation )
//{
//    Eigen::Matrix3d _rotation3d;

//    _rotation3d = rotation.block( 0 , 0 , 3 , 3 );

//    Eigen::Quaterniond _quaternion;
//    _quaternion = _rotation3d;

//    return _quaternion;
//}

//Eigen::Vector3d RosTextDisplayNode::quaternion2rpy(const Eigen::Quaterniond &quaternion )
//{
//    Eigen::Vector3d _rpy = rotation2rpy( quaternion.toRotationMatrix() );

//    return _rpy;
//}

//Eigen::Vector3d RosTextDisplayNode::quaternion2rpy(const geometry_msgs::Quaternion &quaternion)
//{
//    Eigen::Quaterniond _quaternion;
//    tf::quaternionMsgToEigen(quaternion, _quaternion);

//    Eigen::Vector3d _rpy = rotation2rpy( _quaternion.toRotationMatrix() );

//    return _rpy;
//}

//Eigen::MatrixXd RosTextDisplayNode::quaternion2rotation(const Eigen::Quaterniond &quaternion )
//{
//    Eigen::MatrixXd _rotation = quaternion.toRotationMatrix();

//    return _rotation;
//}

//Eigen::MatrixXd RosTextDisplayNode::rotationX(const double &angle )
//{
//    Eigen::MatrixXd _rotation( 3 , 3 );

//    _rotation << 1.0,          0.0,           0.0,
//                 0.0, cos( angle ), -sin( angle ),
//                 0.0, sin( angle ),  cos( angle );

//    return _rotation;
//}

//Eigen::MatrixXd RosTextDisplayNode::rotationY(const double &angle )
//{
//    Eigen::MatrixXd _rotation( 3 , 3 );

//    _rotation << cos( angle ), 0.0, sin( angle ),
//                          0.0, 1.0,          0.0,
//                -sin( angle ), 0.0, cos( angle );

//    return _rotation;
//}

//Eigen::MatrixXd RosTextDisplayNode::rotationZ(const double &angle )
//{
//    Eigen::MatrixXd _rotation(3,3);

//    _rotation << cos( angle ), -sin( angle ), 0.0,
//                 sin( angle ),  cos( angle ), 0.0,
//                          0.0,           0.0, 1.0;

//    return _rotation;
//}

} // end namespace rviz_plugin_thor
