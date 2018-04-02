#ifndef RVIZ_POSE_PANEL_H
#define RVIZ_POSE_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
#endif

#include <std_msgs/String.h>
//#include <geometry_msgs/Pose.h>

//#include <Eigen/Dense>
//#include <eigen_conversions/eigen_msg.h>

#include "ui_rviz_text_display.h"
#include "rviz_text_display/ros_text_display_node.h"

namespace rviz_robotis
{

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Panel. Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.

class TextDisplayPanel: public rviz::Panel
{
        // This class uses Qt slots and is a subclass of QObject, so it needs
        // the Q_OBJECT macro.
        Q_OBJECT
    public:
        // QWidget subclass constructors usually take a parent widget
        // parameter (which usually defaults to 0). At the same time,
        // pluginlib::ClassLoader creates instances by calling the default
        // constructor (with no arguments). Taking the parameter and giving
        // a default of 0 lets the default constructor work and also lets
        // someone using the class for something else to pass in a parent
        // widget as they normally would with Qt.
        TextDisplayPanel( QWidget* parent = 0 );

        // Now we declare overrides of rviz::Panel functions for saving and
        // loading data from the config file. Here the data is the
        // topic name.
//        virtual void load( const rviz::Config& config );
//        virtual void save( rviz::Config config ) const;

    // Next come a couple of public Qt slots.
    public Q_SLOTS:
//        void on_doubleSpinBox_position_x_valueChanged(double value);
//        void on_doubleSpinBox_position_y_valueChanged(double value);
//        void on_doubleSpinBox_position_z_valueChanged(double value);
//        void on_button_publish_position_clicked();

//        void on_doubleSpinBox_orientation_r_valueChanged(double value);
//        void on_doubleSpinBox_orientation_p_valueChanged(double value);
//        void on_doubleSpinBox_orientation_y_valueChanged(double value);
//        void on_button_publish_pose_clicked();

//        void on_button_set_marker_clicked();
//        void on_button_get_marker_clicked();
//        void on_button_clear_marker_clicked();

//        void updatePointPanel(const Point_msg point);
//        void updatePosePanel(const Pose_msg pose);
        void updateTextPanel(const QString msg );

    // Here we declare some internal slots.
    protected Q_SLOTS:

    Q_SIGNALS:

    protected:
        Ui::TextPanelForm ui;
        RosTextDisplayNode ros_node_;

//        bool from_UI;

//        void getPoseFromPanel(geometry_msgs::Pose &current);
//        void setPoseToPanel(const geometry_msgs::Pose &current);
//        void getPointFromPanel(geometry_msgs::Point &current);
//        void setPointToPanel(const geometry_msgs::Point &current);

//        void makeInteractiveMarker();
//        void updateInteractiveMarker();
//        void clearPanel();
//        void publishCurrentPoint();
//        void publishCurrentPose();
};

} // end namespace rviz_plugin_thor
#endif // RVIZ_POSE_PANEL_H
