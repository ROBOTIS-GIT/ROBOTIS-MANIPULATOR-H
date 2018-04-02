#ifndef DEMO_MODULE_ROBOTISCOMMON_H_
#define DEMO_MODULE_ROBOTISCOMMON_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT
#include <Eigen/Dense>

namespace ROBOTIS_DEMO
{

#define MAX_JOINT_ID	7
#define ALL_JOINT_ID    8

#define MAX_ITER        10

#define end_effector    8

#define deg2rad 	(M_PI / 180.0)
#define rad2deg 	(180.0 / M_PI)

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

}

#endif /* DEMO_MODULE_ROBOTISCOMMON_H_ */
