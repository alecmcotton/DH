#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <Eigen/Dense>
#include <vector>

namespace kinematics {

Eigen::Matrix4d forwardKinematics(
    const std::vector<Eigen::VectorXd>& dh_table,
    const Eigen::VectorXd& joint_angles);

Eigen::VectorXd poseFromTransformation(const Eigen::Matrix4d& T);

bool inverseKinematics(
    const std::vector<Eigen::VectorXd>& dh_table,
    const Eigen::VectorXd& desired_pose,
    Eigen::VectorXd& initial_guess,
    int max_iters = 100,
    double tol = 1e-6);

}

#endif
