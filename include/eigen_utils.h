#pragma once
#include <Eigen/Dense>

namespace dh_kinematics {

// Convert rotation matrix to Euler angles (ZYX convention)
Eigen::Vector3d rotationMatrixToEulerAngles(const Eigen::Matrix3d& R);

// Convert Euler angles (ZYX convention) to rotation matrix
Eigen::Matrix3d eulerAnglesToRotationMatrix(const Eigen::Vector3d& euler);

} // namespace dh_kinematics
