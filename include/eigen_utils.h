#pragma once
#include <Eigen/Dense>

namespace dh_kinematics {

Eigen::Vector3d rotationMatrixToEulerAngles(const Eigen::Matrix3d& R);

Eigen::Matrix3d eulerAnglesToRotationMatrix(const Eigen::Vector3d& euler);

}
