#include "kinematics.hpp"
#include <iostream>

using namespace Eigen;

namespace kinematics {

static Matrix4d dhTransform(const VectorXd& dh_params) {
    double alpha = dh_params(0);
    double a = dh_params(1);
    double d = dh_params(2);
    double theta = dh_params(3);

    Matrix4d A;
    A << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta),
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
                  0,             sin(alpha),             cos(alpha),           d,
                  0,                      0,                      0,           1;
    return A;
}

Matrix4d forwardKinematics(const std::vector<VectorXd>& dh_table, const VectorXd& joint_angles) {
    Matrix4d T = Matrix4d::Identity();

    if (dh_table.size() != joint_angles.size()) {
        throw std::runtime_error("DH table and joint angles size mismatch");
    }

    for (size_t i = 0; i < dh_table.size(); ++i) {
        VectorXd dh = dh_table[i];
        VectorXd dh_mod = dh;
        dh_mod(3) = joint_angles(i);

        T *= dhTransform(dh_mod);
    }
    return T;
}

VectorXd poseFromTransformation(const Matrix4d& T) {
    VectorXd pose(6);
    pose.head(3) = T.block<3,1>(0,3);

    double roll, pitch, yaw;
    pitch = asin(-T(2,0));
    if (cos(pitch) > 1e-6) {
        roll = atan2(T(2,1), T(2,2));
        yaw = atan2(T(1,0), T(0,0));
    } else {
        roll = 0;
        yaw = atan2(-T(0,1), T(1,1));
    }

    pose(3) = roll;
    pose(4) = pitch;
    pose(5) = yaw;
    return pose;
}

bool inverseKinematics(const std::vector<VectorXd>& dh_table,
                       const VectorXd& desired_pose,
                       VectorXd& joint_angles,
                       int max_iters,
                       double tol) {
    size_t n = dh_table.size();

    if (joint_angles.size() != n) {
        joint_angles = VectorXd::Zero(n);
    }

    for (int iter = 0; iter < max_iters; ++iter) {
        Matrix4d T = forwardKinematics(dh_table, joint_angles);
        VectorXd current_pose = poseFromTransformation(T);
        VectorXd error = desired_pose - current_pose;

        if (error.norm() < tol) {
            return true;
        }

        MatrixXd J(6, n);
        double delta = 1e-6;
        for (size_t i = 0; i < n; ++i) {
            VectorXd joint_plus = joint_angles;
            joint_plus(i) += delta;

            VectorXd pose_plus = poseFromTransformation(forwardKinematics(dh_table, joint_plus));
            J.col(i) = (pose_plus - current_pose) / delta;
        }

        double lambda = 0.01;
        MatrixXd JTJ = J.transpose() * J + lambda * MatrixXd::Identity(n, n);
        VectorXd delta_theta = JTJ.ldlt().solve(J.transpose() * error);

        joint_angles += delta_theta;
    }

    return false;
}

}
