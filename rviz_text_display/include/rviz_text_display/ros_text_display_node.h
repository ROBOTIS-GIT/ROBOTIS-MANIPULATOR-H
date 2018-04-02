#ifndef ROS_POSE_PANEL_NODE_H
#define ROS_POSE_PANEL_NODE_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
#endif

#include <QThread>

#include <std_msgs/String.h>

//#include <Eigen/Dense>
//#include <eigen_conversions/eigen_msg.h>

//#include <interactive_markers/interactive_marker_server.h>
//#include <visualization_msgs/InteractiveMarker.h>
//#include <geometry_msgs/PointStamped.h>

namespace rviz_robotis
{

// for connection
//typedef geometry_msgs::Pose Pose_msg;
//typedef geometry_msgs::Point Point_msg;

class RosTextDisplayNode : public QThread
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
        RosTextDisplayNode();
        ~RosTextDisplayNode();

        bool init();
        void run();

        // Now we declare overrides of rviz::Panel functions for saving and
        // loading data from the config file. Here the data is the
        // topic name.
//        void makeInteractiveMarker(const geometry_msgs::Pose &pose);
//        void updateInteractiveMarker(const geometry_msgs::Pose &pose);
//        void clearInteractiveMarker();
//        void publishPoint(const geometry_msgs::Point &point);
//        void publishPose(const geometry_msgs::Pose &pose);

//        Eigen::Vector3d rotation2rpy(const Eigen::MatrixXd &rotation );
//        Eigen::MatrixXd rpy2rotation(const double &roll,const double &pitch,const double &yaw );
//        Eigen::Quaterniond rpy2quaternion(const Eigen::Vector3d &euler);
//        Eigen::Quaterniond rpy2quaternion(const double &roll,const double &pitch,const double &yaw );
//        Eigen::Quaterniond rotation2quaternion(const Eigen::MatrixXd &rotation );
//        Eigen::Vector3d quaternion2rpy(const Eigen::Quaterniond &quaternion );
//        Eigen::Vector3d quaternion2rpy(const geometry_msgs::Quaternion &quaternion);
//        Eigen::MatrixXd quaternion2rotation(const Eigen::Quaterniond &quaternion );
//        Eigen::MatrixXd rotationX(const double &angle );
//        Eigen::MatrixXd rotationY(const double &angle );
//        Eigen::MatrixXd rotationZ(const double &angle );

        // Next come a couple of public Qt slots.
    public Q_SLOTS:
//        void getInteractiveMarkerPose();

        // Here we declare some internal slots.
    protected Q_SLOTS:

    Q_SIGNALS:
        // void rosShutdown();
//        void updateCurrentPoint(const Point_msg point);
//        void updateCurrentPose(const Pose_msg pose);

        void updateCurrentText( const QString msg );

    protected:

        // The current name of the output topic.
        // QString output_topic_;
        // The ROS publisher for the command velocity.
//        ros::Publisher pose_pub_;
//        ros::Publisher point_pub_;
//        ros::Subscriber clicked_point_sub_;

        ros::Subscriber current_text_sub_;

        // The ROS node handle.
        ros::NodeHandle nh_;

//        std::string frame_id_;
//        std::string marker_name_;
//        geometry_msgs::Pose current_pose_;
//        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;

//        void interactiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
//        void pointStampedCallback(const geometry_msgs::PointStamped::ConstPtr &msg);

        void CurrentTextCallback( const std_msgs::String::ConstPtr & msg );

};

//template <typename T> T deg2rad(T deg)
//{
//    return deg * M_PI / 180;
//}

//template <typename T> T rad2deg(T rad)
//{
//    return rad * 180 / M_PI;
//}

} // end namespace rviz_thor3
#endif // ROS_POSE_PANEL_NODE_H
