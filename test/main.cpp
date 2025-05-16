#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Dense>
#include "kinematics.hpp"

using namespace std;
using namespace Eigen;

int main() {
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_int_distribution<> n_joints_dist(2, 10);   // 2 to 10 joints
    std::uniform_real_distribution<> alpha_dist(-M_PI, M_PI);
    std::uniform_real_distribution<> a_dist(-1.0, 1.0);
    std::uniform_real_distribution<> d_dist(0.0, 1.0);
    std::uniform_real_distribution<> theta_dist(-M_PI, M_PI);

    int trials = 10;
    for (int trial = 0; trial < trials; ++trial) {
        int n_joints = n_joints_dist(gen);
        vector<VectorXd> dh_table;

        for (int i = 0; i < n_joints; ++i) {
            VectorXd dh(4);
            dh << alpha_dist(gen), a_dist(gen), d_dist(gen), 0.0; // set initial theta = 0
            dh_table.push_back(dh);
        }

        VectorXd joint_values(n_joints);
        for (int i = 0; i < n_joints; ++i) {
            joint_values(i) = theta_dist(gen);
        }

        Matrix4d fk_transform = kinematics::forwardKinematics(dh_table, joint_values);

        VectorXd pose = kinematics::poseFromTransformation(fk_transform);

        VectorXd ik_guess = VectorXd::Zero(n_joints);

        VectorXd ik_result = ik_guess;
        bool converged = kinematics::inverseKinematics(dh_table, pose, ik_result);

        cout << "Trial " << trial + 1 << " / " << trials << "\n";
        cout << "Number of joints: " << n_joints << "\n";
        cout << "Original joint values: " << joint_values.transpose() << "\n";
        cout << "IK converged: " << (converged ? "Yes" : "No") << "\n";
        if (converged) {
            cout << "IK result: " << ik_result.transpose() << "\n";

            Matrix4d fk_check = kinematics::forwardKinematics(dh_table, ik_result);
            VectorXd pose_check = kinematics::poseFromTransformation(fk_check);

            cout << "Pose error norm: " << (pose_check - pose).norm() << "\n";
        }
        cout << "----------------------------------\n";
    }

    return 0;
}
