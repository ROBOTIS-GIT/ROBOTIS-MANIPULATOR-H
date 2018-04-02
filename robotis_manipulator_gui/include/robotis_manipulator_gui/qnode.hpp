/**
 * @file /include/robotis_manipulator_h_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robotis_manipulator_gui_QNODE_HPP_
#define robotis_manipulator_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

#include <Eigen/Dense>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include <robotis_controller_msgs/JointCtrlModule.h>
#include <robotis_controller_msgs/SyncWriteItem.h>

#include <manipulator_demo_module_msgs/JointPose.h>
#include <manipulator_demo_module_msgs/KinematicsPose.h>
#include <manipulator_demo_module_msgs/RobotisDemo.h>

#include <manipulator_demo_module_msgs/GetJointPose.h>
#include <manipulator_demo_module_msgs/GetKinematicsPose.h>

#endif

#define tol_cnt     80

#define deg2rad 	(M_PI / 180.0)
#define rad2deg 	(180.0 / M_PI)

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robotis_manipulator_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
    void log( const LogLevel &level, const std::string &msg );

    void sendIniPoseMsg( std_msgs::String msg );
    void sendSetModeMsg( std_msgs::String msg );

    void sendSyncSWriteItemMsg( robotis_controller_msgs::SyncWriteItem msg );

    void sendJointPoseMsg( manipulator_demo_module_msgs::JointPose msg );
    void sendKinematicsPoseMsg( manipulator_demo_module_msgs::KinematicsPose msg );
    void sendExecuteMsg( std_msgs::String msg );

    void sendRobotisDemoMsg( std_msgs::String msg );

    void laserOn();
    void laserOff();

    Eigen::MatrixXd rotationX( double angle );
    Eigen::MatrixXd rotationY( double angle );
    Eigen::MatrixXd rotationZ( double angle );
    Eigen::Quaterniond rpy2quaternion( double roll, double pitch, double yaw );
    Eigen::MatrixXd quaternion2rpy( Eigen::Quaterniond quaternion );

    bool is_demo_running;
    std_msgs::String demo_name;

    int ar_id;
    int cnt;
    int cnt_1 , cnt_2;

    bool send_ar_pose;
    bool get_ar_pose;
    bool half_cycle;
    geometry_msgs::Pose ar_pose_msg;

    bool send_ar_1_pose;
    bool get_ar_1_pose;
    geometry_msgs::Pose ar_1_pose_msg;

    bool send_ar_2_pose;
    bool get_ar_2_pose;
    geometry_msgs::Pose ar_2_pose_msg;

    double ar_1_x_offset;
    double ar_1_y_offset;
    double ar_2_x_offset;
    double ar_2_y_offset;

public Q_SLOTS:
    void getJointPose( std::vector<std::string> joint_name );
    void getKinematicsPose ( std::string group_name );
    void RobotisDemoMsgCallback( std_msgs::String after_robotis_demo_msg );

    void AR1PoseCallback( geometry_msgs::Pose ar_pose_msg );
    void AR2PoseCallback( geometry_msgs::Pose ar_pose_msg );

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

    void update_curr_joint_pose( manipulator_demo_module_msgs::JointPose );
    void update_curr_kinematics_pose( manipulator_demo_module_msgs::KinematicsPose );

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;

    ros::Publisher ini_pose_msg_pub;
    ros::Publisher set_mode_msg_pub;

    ros::Publisher joint_pose_msg_pub;
    ros::Publisher kinematics_pose_msg_pub;

    ros::Publisher syncwrite_item_pub_;

    ros::Publisher current_text_pub;

    ros::Publisher execute_planned_path_msg_pub;

    ros::ServiceClient get_joint_pose_client;
    ros::ServiceClient get_kinematics_pose_client;

    ros::Publisher robotis_demo_msg_pub;
    ros::Subscriber robotis_demo_msg_sub;
    ros::Subscriber ar_1_pose_sub;
    ros::Subscriber ar_2_pose_sub;

};

}  // namespace robotis_manipulator_gui

#endif /* robotis_manipulator_gui_QNODE_HPP_ */
