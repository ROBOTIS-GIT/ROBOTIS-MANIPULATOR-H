#ifndef DEMO_MODULE_ROBOTISDATA_H_
#define DEMO_MODULE_ROBOTISDATA_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <vector>

#include "RobotisCommon.h"
#include "RobotisLink.h"

namespace ROBOTIS_DEMO
{

class RobotisData
{

public:

    RobotisLink *robotis_joint [ ALL_JOINT_ID + 1 ];

    RobotisData();
    ~RobotisData();
    RobotisData(TREE_SELECT tree);

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

#endif /* DEMO_MODULE_ROBOTISDATA_H_ */
