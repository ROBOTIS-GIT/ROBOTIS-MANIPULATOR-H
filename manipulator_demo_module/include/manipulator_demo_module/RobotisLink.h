#ifndef DEMO_MODULE_ROBOTISLINK_H_
#define DEMO_MODULE_ROBOTISLINK_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT
#include <Eigen/Dense>

namespace ROBOTIS_DEMO
{

enum TREE_SELECT {
    ARM
};

class RobotisLink
{

public:

    RobotisLink();
    ~RobotisLink();

    std::string name;

    int parent;
    int sibling;
    int child;

    double mass;

    Eigen::MatrixXd relative_position;
    Eigen::MatrixXd joint_axis;
    Eigen::MatrixXd center_of_mass;
    Eigen::MatrixXd inertia;

    double joint_limit_max;
    double joint_limit_min;

    double joint_angle;
    double joint_velocity;
    double joint_acceleration;

    Eigen::MatrixXd position;
    Eigen::MatrixXd orientation;
    Eigen::MatrixXd transformation;

};

}

#endif /* DEMO_MODULE_ROBOTISLINK_H_ */
