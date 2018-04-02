#ifndef MANIPULATOR_KINEMATICS_DYNAMICS_H_
#define MANIPULATOR_KINEMATICS_DYNAMICS_H_

#include <vector>

#include "ManipulatorKinematicsDynamicsDefine.h"
#include "LinkData.h"

namespace ROBOTIS
{

enum TREE_SELECT {
    ARM
};

class ManipulatorKinematicsDynamics
{

public:

    LinkData *manipulator_link_data [ ALL_JOINT_ID + 1 ];

	ManipulatorKinematicsDynamics();
    ~ManipulatorKinematicsDynamics();
    ManipulatorKinematicsDynamics(TREE_SELECT tree);

    std::vector<int> findRoute( int to );
    std::vector<int> findRoute( int from , int to );

    double totalMass( int joint_ID );
    Eigen::MatrixXd calcMC( int joint_ID );
    Eigen::MatrixXd calcCOM( Eigen::MatrixXd MC );

    void forwardKinematics( int joint_ID );

    Eigen::MatrixXd calcJacobian( std::vector<int> idx );
    Eigen::MatrixXd calcJacobianCOM( std::vector<int> idx );
    Eigen::MatrixXd calcVWerr( Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position, Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation );

    bool inverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation , int max_iter, double ik_err);
    bool inverseKinematics( int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err );
};

}

#endif /* MANIPULATOR_KINEMATICS_DYNAMICS_H_ */
