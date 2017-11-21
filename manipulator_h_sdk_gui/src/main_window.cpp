/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "manipulator_h_sdk_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace manipulator_h_sdk_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  joint_name.push_back("joint1");
  joint_name.push_back("joint2");
  joint_name.push_back("joint3");
  joint_name.push_back("joint4");
  joint_name.push_back("joint5");
  joint_name.push_back("joint6");

  /*********************
    ** Logging
    **********************/
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  joint_box.append( ui.a1_box );
  joint_box.append( ui.a2_box );
  joint_box.append( ui.a3_box );
  joint_box.append( ui.a4_box );
  joint_box.append( ui.a5_box );
  joint_box.append( ui.a6_box );

  /****************************
    ** Connect
    ****************************/

  qRegisterMetaType<manipulator_h_sdk::JointPose>("manipulator_h_sdk::JointPose");
  QObject::connect(&qnode, SIGNAL(updateCurrentJointPose(manipulator_h_sdk::JointPose)), this, SLOT(updateCurrJointPoseSpinbox(manipulator_h_sdk::JointPose)));

  qRegisterMetaType<manipulator_h_sdk::KinematicsPose>("manipulator_h_sdk::KinematicsPose");
  QObject::connect(&qnode, SIGNAL(updateCurrentKinematicsPose(manipulator_h_sdk::KinematicsPose)), this, SLOT(updateCurrKinematicsPoseSpinbox(manipulator_h_sdk::KinematicsPose)));

  /*********************
    ** Auto Start
    **********************/
  qnode.init();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::on_set_mode_button_clicked( bool check )
{
  qnode.sendSetModeMsg();
}

void MainWindow::on_joint_get_box_clicked( bool check )
{
  qnode.getJointPose();
}

void MainWindow::on_joint_send_box_clicked( bool check )
{
  manipulator_h_sdk::JointPose msg;
  msg.mov_time = ui.joint_mov_time_box->value();

  for ( int _id = 0; _id < joint_box.size(); _id++ )
  {
    msg.pose.name.push_back( joint_name[ _id ] );
    msg.pose.position.push_back( ((QDoubleSpinBox *) joint_box[ _id ])->value() * M_PI / 180.0 );
  }

  qnode.sendJointPoseMsg( msg );
}

void MainWindow::on_init_pose_button_clicked( bool check )
{
  std::string ini_pose_path = ros::package::getPath("manipulator_h_sdk_gui") + "/config/init_pose.yaml";
  parseIniPoseData(ini_pose_path);

  std_msgs::Bool msg;
  msg.data = true;
}

void MainWindow::on_zero_pose_button_clicked( bool check )
{
  std::string ini_pose_path = ros::package::getPath("manipulator_h_sdk_gui") + "/config/zero_pose.yaml";
  parseIniPoseData(ini_pose_path);
}

void MainWindow::on_task_send_button_clicked( bool check )
{
  manipulator_h_sdk::KinematicsPose msg;

  msg.name = ui.task_group_name_box->currentText().toStdString();

  msg.mov_time = ui.task_mov_time_box->value();

  msg.pose.position.x = ui.goal_x_box->value();
  msg.pose.position.y = ui.goal_y_box->value();
  msg.pose.position.z = ui.goal_z_box->value();

  double roll = ui.goal_roll_box->value() * DEG2RAD;
  double pitch = ui.goal_pitch_box->value() * DEG2RAD;
  double yaw = ui.goal_yaw_box->value() * DEG2RAD;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  qnode.sendKinematicsPoseMsg( msg );
}

void MainWindow::on_task_send_button_2_clicked( bool check )
{
  manipulator_h_sdk::KinematicsPose msg;

  msg.name = ui.task_group_name_box->currentText().toStdString();

  msg.mov_time = ui.task_mov_time_box->value();

  msg.pose.position.x = ui.present_x_box->value();
  msg.pose.position.y = ui.present_y_box->value();
  msg.pose.position.z = ui.present_z_box->value();

  double roll = ui.present_roll_box->value() * DEG2RAD;
  double pitch = ui.present_pitch_box->value() * DEG2RAD;
  double yaw = ui.present_yaw_box->value() * DEG2RAD;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  qnode.sendKinematicsPoseMsg( msg );
}


void MainWindow::on_task_get_button_clicked( bool check )
{
  std::string group_name = ui.task_group_name_box->currentText().toStdString();
  qnode.getKinematicsPose(group_name);
}

void MainWindow::updateCurrJointPoseSpinbox( manipulator_h_sdk::JointPose msg )
{
  for ( int i=0; i<msg.pose.name.size(); i++ )
    ((QDoubleSpinBox *) joint_box[i])->setValue( msg.pose.position[i] * RAD2DEG );
}

void MainWindow::updateCurrKinematicsPoseSpinbox( manipulator_h_sdk::KinematicsPose msg )
{
  ui.goal_x_box->setValue( msg.pose.position.x );
  ui.goal_y_box->setValue( msg.pose.position.y );
  ui.goal_z_box->setValue( msg.pose.position.z );

  Eigen::Quaterniond QR( msg.pose.orientation.w , msg.pose.orientation.x , msg.pose.orientation.y , msg.pose.orientation.z );
  Eigen::MatrixXd rpy = quaternion2rpy( QR );

  double roll = rpy.coeff( 0 , 0 ) * 180.0 / M_PI;
  double pitch = rpy.coeff( 1 , 0 ) * 180.0 / M_PI;
  double yaw = rpy.coeff( 2, 0 ) * 180.0 /M_PI;

  ui.goal_roll_box->setValue( roll );
  ui.goal_pitch_box->setValue( pitch );
  ui.goal_yaw_box->setValue( yaw );
}

Eigen::MatrixXd MainWindow::rotationX( double angle )
{
  Eigen::MatrixXd rotation(3,3);

  rotation << 1.0, 0.0, 0.0,
              0.0, cos(angle), -sin(angle),
              0.0, sin(angle),  cos(angle);

  return rotation;
}

Eigen::MatrixXd MainWindow::rotationY( double angle )
{
  Eigen::MatrixXd rotation(3,3);

  rotation << cos(angle), 0.0, sin(angle),
              0.0, 1.0, 0.0,
              -sin(angle), 0.0, cos(angle);

  return rotation;
}

Eigen::MatrixXd MainWindow::rotationZ( double angle )
{
  Eigen::MatrixXd rotation(3,3);

  rotation << cos(angle), -sin(angle), 0.0,
              sin(angle),  cos(angle), 0.0,
              0.0, 0.0, 1.0;

  return rotation;
}

Eigen::MatrixXd MainWindow::rotation2rpy( Eigen::MatrixXd rotation )
{
  Eigen::MatrixXd rpy = Eigen::MatrixXd::Zero(3,1);

  rpy.coeffRef(0,0) = atan2( rotation.coeff(2,1), rotation.coeff(2,2) );
  rpy.coeffRef(1,0) = atan2(-rotation.coeff(2,0), sqrt(pow(rotation.coeff(2,1),2) + pow(rotation.coeff(2,2),2)));
  rpy.coeffRef(2,0) = atan2( rotation.coeff(1,0), rotation.coeff(0,0) );

  return rpy;
}

Eigen::MatrixXd MainWindow::rpy2rotation( double roll, double pitch, double yaw )
{
  Eigen::MatrixXd rotation = rotationZ(yaw) * rotationY(pitch) * rotationX(roll);

  return rotation;
}

Eigen::Quaterniond MainWindow::rpy2quaternion( double roll, double pitch, double yaw )
{
  Eigen::MatrixXd rotation = rpy2rotation(roll,pitch,yaw);

  Eigen::Matrix3d rotation3d;
  rotation3d = rotation.block(0,0,3,3);

  Eigen::Quaterniond quaternion;

  quaternion = rotation3d;

  return quaternion;
}

Eigen::Quaterniond MainWindow::rotation2quaternion( Eigen::MatrixXd rotation )
{
  Eigen::Matrix3d rotation3d;

  rotation3d = rotation.block(0,0,3,3);

  Eigen::Quaterniond quaternion;
  quaternion = rotation3d;

  return quaternion;
}

Eigen::MatrixXd MainWindow::quaternion2rpy( Eigen::Quaterniond quaternion )
{
  Eigen::MatrixXd rpy = rotation2rpy(quaternion.toRotationMatrix());

  return rpy;
}

Eigen::MatrixXd MainWindow::quaternion2rotation( Eigen::Quaterniond quaternion )
{
  Eigen::MatrixXd rotation = quaternion.toRotationMatrix();

  return rotation;
}

void MainWindow::parseIniPoseData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  manipulator_h_sdk::JointPose msg;

  // parse movement time
  double mov_time = doc["mov_time"].as<double>();
  msg.mov_time = mov_time;

  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it)
  {
    std::string joint_name = it->first.as<std::string>();
    double value = it->second.as<double>();

    msg.pose.name.push_back(joint_name);
    msg.pose.position.push_back(value * DEG2RAD);
  }

  qnode.sendJointPoseMsg( msg );
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
  ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
  QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Robotis</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

}  // namespace manipulator_h_sdk_gui

