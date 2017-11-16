#include <stdio.h>
#include "manipulator_h_sdk/arm_kinematics.h"

using namespace manipulator_h;

ManipulatorH::ManipulatorH()
{
  arm_joint_position_.resize(NUM_OF_JOINTS);

  for (int i=0; i<NUM_OF_JOINTS; i++)
    arm_joint_position_(i) = 0.0;
}

ManipulatorH::~ManipulatorH()
{

}

void ManipulatorH::initialize()
{
  KDL::Chain arm_chain;

  // Set Kinematics Tree
  arm_chain.addSegment(KDL::Segment("base",
                                     KDL::Joint(KDL::Joint::None),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, 0.126)),
                                     KDL::RigidBodyInertia(0.85644,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  arm_chain.addSegment(KDL::Segment("joint1",
                                     KDL::Joint(KDL::Joint::RotZ),
                                     KDL::Frame(KDL::Vector(0.0, 0.069, 0.033)),
                                     KDL::RigidBodyInertia(0.94658,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  arm_chain.addSegment(KDL::Segment("joint2",
                                     KDL::Joint(KDL::Joint::RotY),
                                     KDL::Frame(KDL::Vector(0.03, -0.0115, 0.264)),
                                     KDL::RigidBodyInertia(1.30260,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  arm_chain.addSegment(KDL::Segment("joint3",
                                     KDL::Joint(KDL::Joint::RotY),
                                     KDL::Frame(KDL::Vector(0.195, -0.0575, 0.03)),
                                     KDL::RigidBodyInertia(1.15977,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  arm_chain.addSegment(KDL::Segment("joint4",
                                     KDL::Joint(KDL::Joint::RotX),
                                     KDL::Frame(KDL::Vector(0.063, 0.045, 0.0)),
                                     KDL::RigidBodyInertia(0.44688,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  arm_chain.addSegment(KDL::Segment("joint5",
                                     KDL::Joint(KDL::Joint::RotY),
                                     KDL::Frame(KDL::Vector(0.123, -0.045, 0.0)),
                                     KDL::RigidBodyInertia(0.43273,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );
  arm_chain.addSegment(KDL::Segment("joint6",
                                     KDL::Joint(KDL::Joint::RotX),
                                     KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                     KDL::RigidBodyInertia(0.01919,
                                                           KDL::Vector(0.0, 0.0, 0.0),
                                                           KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                           )
                                     )
                        );

  // Set Joint Limits
  std::vector<double> min_position_limit, max_position_limit;
  min_position_limit.push_back(-180.0); max_position_limit.push_back(180.0); // joint1
  min_position_limit.push_back(-180.0);	max_position_limit.push_back(180.0); // joint2
  min_position_limit.push_back(-180.0); max_position_limit.push_back(180.0); // joint3
  min_position_limit.push_back(-180.0);	max_position_limit.push_back(180.0); // joint4
  min_position_limit.push_back(-180.0);	max_position_limit.push_back(180.0); // joint5
  min_position_limit.push_back(-180.0);	max_position_limit.push_back(180.0); // joint6

  KDL::JntArray min_joint_limit(NUM_OF_JOINTS), max_joint_limit(NUM_OF_JOINTS);
  for (int index=0; index<NUM_OF_JOINTS; index++)
  {
    min_joint_limit(index) = min_position_limit[index]*D2R;
    max_joint_limit(index) = max_position_limit[index]*D2R;
  }

  /* KDL Solver Initialization */

  // forward kinematics solver
  arm_fk_solver_ = new KDL::ChainFkSolverPos_recursive(arm_chain);

  // inverse kinematics solver
  arm_ik_vel_solver_ = new KDL::ChainIkSolverVel_pinv(arm_chain);
  arm_ik_pos_solver_ = new KDL::ChainIkSolverPos_NR_JL(arm_chain,
                                                       min_joint_limit, max_joint_limit,
                                                       *arm_fk_solver_,
                                                       *arm_ik_vel_solver_);


}

void ManipulatorH::setJointPosition(Eigen::VectorXd arm_joint_position)
{
  arm_joint_position_ = arm_joint_position;
}

void ManipulatorH::solveForwardKinematics(std::vector<double_t> &arm_position, std::vector<double_t> &arm_orientation)
{
  KDL::JntArray arm_joint_position;
  arm_joint_position.data = arm_joint_position_;

  KDL::Frame arm_pose;
  arm_fk_solver_->JntToCart(arm_joint_position, arm_pose);

  arm_pose_.position.x = arm_pose.p.x();
  arm_pose_.position.y = arm_pose.p.y();
  arm_pose_.position.z = arm_pose.p.z();

  arm_pose.M.GetQuaternion(arm_pose_.orientation.x,
                           arm_pose_.orientation.y,
                           arm_pose_.orientation.z,
                           arm_pose_.orientation.w);

//  ROS_INFO("arm position x : %f y: %f, z: %f", arm_pose_.position.x, arm_pose_.position.y, arm_pose_.position.z);

  arm_position.resize(3,0.0);
  arm_position[0] = arm_pose_.position.x;
  arm_position[1] = arm_pose_.position.y;
  arm_position[2] = arm_pose_.position.z;

  arm_orientation.resize(4,0.0);
  arm_orientation[0] = arm_pose_.orientation.x;
  arm_orientation[1] = arm_pose_.orientation.y;
  arm_orientation[2] = arm_pose_.orientation.z;
  arm_orientation[3] = arm_pose_.orientation.w;
}

bool ManipulatorH::solveInverseKinematics(std::vector<double_t> &arm_output,
                                          Eigen::MatrixXd arm_target_position, Eigen::Quaterniond arm_target_orientation)
{
  KDL::JntArray arm_joint_position;
  arm_joint_position.data = arm_joint_position_;

  KDL::Frame arm_desired_pose;
  arm_desired_pose.p.x(arm_target_position.coeff(0,0));
  arm_desired_pose.p.y(arm_target_position.coeff(1,0));
  arm_desired_pose.p.z(arm_target_position.coeff(2,0));

  arm_desired_pose.M = KDL::Rotation::Quaternion(arm_target_orientation.x(),
                                                 arm_target_orientation.y(),
                                                 arm_target_orientation.z(),
                                                 arm_target_orientation.w());

  KDL::JntArray arm_desired_joint_position;
  arm_desired_joint_position.resize(NUM_OF_JOINTS);

  int arm_err = arm_ik_pos_solver_->CartToJnt(arm_joint_position, arm_desired_pose, arm_desired_joint_position);

  if (arm_err < 0)
  {
    ROS_WARN("IK ERR : %d", arm_err);
    return false;
  }

  // output
  arm_output.resize(NUM_OF_JOINTS);

  for (int i=0; i<NUM_OF_JOINTS; i++)
    arm_output[i] = arm_desired_joint_position(i);

  return true;
}

void ManipulatorH::finalize()
{
  delete arm_fk_solver_;
  delete arm_ik_vel_solver_;
  delete arm_ik_pos_solver_;
}

