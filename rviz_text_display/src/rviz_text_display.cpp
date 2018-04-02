
#include <stdio.h>

#include "rviz_text_display/rviz_text_display.h"

#include <cmath>

namespace rviz_robotis
{
// BEGIN_TUTORIAL
//
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
TextDisplayPanel::TextDisplayPanel( QWidget* parent )
    : rviz::Panel( parent )
    , ros_node_()
//    , from_UI(true)
{
    // SetUI : *.ui
    // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    ui.setupUi(this);   // auto connect

    // for connection
//    qRegisterMetaType<Pose_msg>("Pose_msg");
//    qRegisterMetaType<Point_msg>("Point_msg");
    // connect SIGNAL & SLOT
//    connect( &ros_node_, SIGNAL(updateCurrentPoint(Point_msg)), this, SLOT(updatePointPanel(Point_msg)));
//    connect( &ros_node_, SIGNAL(updateCurrentPose(Pose_msg)), this, SLOT(updatePosePanel(Pose_msg)));

    connect( &ros_node_, SIGNAL(updateCurrentText(QString)), this, SLOT(updateTextPanel(QString)) );
}

// Save all configuration data from this panel to the given
// Config object. It is important here that you call save()
// on the parent class so the class id and panel name get saved.
//void TextDisplayPanel::save( rviz::Config config ) const
//{
//    rviz::Panel::save( config );
//    // config.mapSetValue( "Topic", output_topic_ );
//}

// Load all configuration data for this panel from the given Config object.
//void TextDisplayPanel::load( const rviz::Config& config )
//{
//    rviz::Panel::load( config );
//    /*
//    QString topic;
//    if( config.mapGetString( "Topic", &topic ))
//    {
//        // output_topic_editor_->setText( topic );
//        updateTopic();
//    }
//    */
//}

// auto-connect SLOT and SIGNAL of UI
//void RobotisTextDisplay::on_doubleSpinBox_position_x_valueChanged(double value) { updateInteractiveMarker(); }
//void RobotisTextDisplay::on_doubleSpinBox_position_y_valueChanged(double value) { updateInteractiveMarker(); }
//void RobotisTextDisplay::on_doubleSpinBox_position_z_valueChanged(double value) { updateInteractiveMarker(); }
//void RobotisTextDisplay::on_button_publish_position_clicked() { publishCurrentPoint(); }

//void RobotisTextDisplay::on_doubleSpinBox_orientation_r_valueChanged(double value) { updateInteractiveMarker(); }
//void RobotisTextDisplay::on_doubleSpinBox_orientation_p_valueChanged(double value) { updateInteractiveMarker(); }
//void RobotisTextDisplay::on_doubleSpinBox_orientation_y_valueChanged(double value) { updateInteractiveMarker(); }
//void RobotisTextDisplay::on_button_publish_pose_clicked() { publishCurrentPose(); }

//void RobotisTextDisplay::on_button_set_marker_clicked() { makeInteractiveMarker(); }
//void RobotisTextDisplay::on_button_get_marker_clicked() { ros_node_.getInteractiveMarkerPose(); }
//void RobotisTextDisplay::on_button_clear_marker_clicked() { clearPanel(); }

// Update UI - position
//void RobotisTextDisplay::updatePointPanel(const Point_msg point)
//{
//    from_UI = false;

//    setPointToPanel(point);

//    ROS_INFO("Update Position Panel");
//    from_UI = true;
//}

//// Update UI - pose
//void RobotisTextDisplay::updatePosePanel(const Pose_msg pose)
//{
//    from_UI = false;

//    setPoseToPanel(pose);

//    ROS_INFO("Update Pose Panel");
//    from_UI = true;
//}

void TextDisplayPanel::updateTextPanel( const QString msg )
{
    ui.text_label->setText( msg );
}

// make interactive marker
//void TextDisplayPanel::makeInteractiveMarker()
//{
//    geometry_msgs::Pose _current_pose;
//    getPoseFromPanel(_current_pose);

//    ros_node_.makeInteractiveMarker(_current_pose);
//}

// update interactive marker pose from ui
//void TextDisplayPanel::updateInteractiveMarker()
//{
//    if(from_UI == false) return;

//    geometry_msgs::Pose _current_pose;
//    getPoseFromPanel(_current_pose);

//    ros_node_.updateInteractiveMarker(_current_pose);
//}

//void TextDisplayPanel::clearPanel()
//{
//    geometry_msgs::Pose _init;
//    updatePosePanel(_init);

//    ROS_INFO("Clear Panel");

//    ros_node_.clearInteractiveMarker();
//}

//void TextDisplayPanel::publishCurrentPoint()
//{
//    geometry_msgs::Point _current_point;
//    getPointFromPanel(_current_point);

//    ros_node_.publishPoint(_current_point);
//}

//void TextDisplayPanel::publishCurrentPose()
//{
//    geometry_msgs::Pose _current_pose;
//    getPoseFromPanel(_current_pose);

//    ros_node_.publishPose(_current_pose);
//}

//void TextDisplayPanel::getPoseFromPanel(geometry_msgs::Pose &current)
//{
//    // position
//    current.position.x = ui.doubleSpinBox_position_x->value();
//    current.position.y = ui.doubleSpinBox_position_y->value();
//    current.position.z = ui.doubleSpinBox_position_z->value();

//    // orientation
//    Eigen::Vector3d _euler(ui.doubleSpinBox_orientation_r->value(),
//                           ui.doubleSpinBox_orientation_p->value(),
//                           ui.doubleSpinBox_orientation_y->value());
//    Eigen::Quaterniond _q = ros_node_.rpy2quaternion(deg2rad<Eigen::Vector3d>(_euler));

//    tf::quaternionEigenToMsg(_q, current.orientation);
//}

//void TextDisplayPanel::setPoseToPanel(const geometry_msgs::Pose &current)
//{
//    // position
//    ui.doubleSpinBox_position_x->setValue(current.position.x);
//    ui.doubleSpinBox_position_y->setValue(current.position.y);
//    ui.doubleSpinBox_position_z->setValue(current.position.z);

//    // orientation
//    Eigen::Vector3d _euler = rad2deg<Eigen::Vector3d>(ros_node_.quaternion2rpy(current.orientation));

//    ui.doubleSpinBox_orientation_r->setValue(_euler[0]);
//    ui.doubleSpinBox_orientation_p->setValue(_euler[1]);
//    ui.doubleSpinBox_orientation_y->setValue(_euler[2]);
//}

//void TextDisplayPanel::getPointFromPanel(geometry_msgs::Point &current)
//{
//    // position
//    current.x = ui.doubleSpinBox_position_x->value();
//    current.y = ui.doubleSpinBox_position_y->value();
//    current.z = ui.doubleSpinBox_position_z->value();
//}

//void TextDisplayPanel::setPointToPanel(const geometry_msgs::Point &current)
//{
//    // position
//    ui.doubleSpinBox_position_x->setValue(current.x);
//    ui.doubleSpinBox_position_y->setValue(current.y);
//    ui.doubleSpinBox_position_z->setValue(current.z);

//    // orientation
//    ui.doubleSpinBox_orientation_r->setValue(0.0);
//    ui.doubleSpinBox_orientation_p->setValue(0.0);
//    ui.doubleSpinBox_orientation_y->setValue(0.0);
//}

} // end namespace rviz_robotis

// Tell pluginlib about this class. Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_robotis::TextDisplayPanel, rviz::Panel )
// END_TUTORIAL
