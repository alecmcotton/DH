#include "dh_kinematics.h"
#include "eigen_utils.h"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <random>

namespace dh_kinematics {

using namespace Eigen;

bool DHKinematics::loadFromCSV(const std::string& filename) {
    dh_params_.clear();
    actuated_joint_indices_.clear();
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    std::string line;
    int joint_index = 0;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        DHParameters params;
        
        std::getline(ss, cell, ',');
        params.a = std::stod(cell);
        std::getline(ss, cell, ',');
        params.alpha = std::stod(cell);
        std::getline(ss, cell, ',');
        params.d = std::stod(cell);
        std::getline(ss, cell, ',');
        params.theta = std::stod(cell);
        
        std::getline(ss, cell, ',');
        if (cell == "R" || cell == "r") {
            params.type = JointType::REVOLUTE;
            actuated_joint_indices_.push_back(joint_index);
        } else if (cell == "P" || cell == "p") {
            params.type = JointType::PRISMATIC;
            actuated_joint_indices_.push_back(joint_index);
        } else {
            params.type = JointType::CONSTANT;
        }
        
        dh_params_.push_back(params);
        joint_index++;
    }
    
    return true;
}

VectorXd DHKinematics::forward(const VectorXd& joint_values) const {
    if (joint_values.size() != getNumJoints()) {
        throw std::runtime_error("Invalid number of joint values");
    }
    
    Matrix4d transform = Matrix4d::Identity();
    VectorXd current_joints = joint_values;
    
    for (size_t i = 0; i < dh_params_.size(); ++i) {
        const auto& params = dh_params_[i];
        double a = params.a;
        double alpha = params.alpha;
        double d = params.d;
        double theta = params.theta;
        
        transform *= computeDHMatrix(a, alpha, d, theta);
    }
    
    return extractPose(transform);
}

VectorXd DHKinematics::inverse(const VectorXd& target_pose, 
                             const VectorXd& initial_guess,
                             double tolerance,
                             int max_iterations) const {
    if (initial_guess.size() != getNumJoints()) {
        throw std::runtime_error("Invalid number of joint values in initial guess");
    }
    
    VectorXd q = initial_guess;
    VectorXd error(6);
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        VectorXd current_pose = forward(q);
        error = target_pose - current_pose;
        
        if (error.norm() < tolerance) {
            break;
        }
        
        MatrixXd J = jacobian(q);
        q += J.jacobiSvd(ComputeThinU | ComputeThinV).solve(error);
    }
    
    return q;
}

MatrixXd DHKinematics::jacobian(const VectorXd& joint_values, double delta) const {
    const int num_joints = getNumJoints();
    MatrixXd J(6, num_joints);
    VectorXd pose = forward(joint_values);
    
    for (int i = 0; i < num_joints; ++i) {
        VectorXd perturbed = joint_values;
        perturbed(i) += delta;
        VectorXd new_pose = forward(perturbed);
        J.col(i) = (new_pose - pose) / delta;
    }
    
    return J;
}

int DHKinematics::getNumJoints() const {
    return actuated_joint_indices_.size();
}

const std::vector<DHParameters>& DHKinematics::getDHParameters() const {
    return dh_params_;
}

void DHKinematics::setDHParameters(const std::vector<DHParameters>& params) {
    dh_params_ = params;
    actuated_joint_indices_.clear();
    for (size_t i = 0; i < dh_params_.size(); ++i) {
        if (dh_params_[i].type != JointType::CONSTANT) {
            actuated_joint_indices_.push_back(i);
        }
    }
}

Matrix4d DHKinematics::computeDHMatrix(double a, double alpha, double d, double theta) const {
    Matrix4d mat;
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    double cos_alpha = cos(alpha);
    double sin_alpha = sin(alpha);
    
    mat << cos_theta, -sin_theta * cos_alpha,  sin_theta * sin_alpha, a * cos_theta,
           sin_theta,  cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta,
           0,          sin_alpha,             cos_alpha,             d,
           0,          0,                     0,                     1;
    
    return mat;
}

VectorXd DHKinematics::extractPose(const Matrix4d& transform) const {
    VectorXd pose(6);
    
    pose(0) = transform(0, 3);
    pose(1) = transform(1, 3);
    pose(2) = transform(2, 3);
    
    Matrix3d R = transform.block<3, 3>(0, 0);
    Vector3d euler = rotationMatrixToEulerAngles(R);
    
    pose(3) = euler(0);
    pose(4) = euler(1);
    pose(5) = euler(2);
    
    return pose;
}

Matrix4d DHKinematics::poseToTransform(const VectorXd& pose) const {
    Matrix4d transform = Matrix4d::Identity();
    
    transform(0, 3) = pose(0);
    transform(1, 3) = pose(1);
    transform(2, 3) = pose(2);
    
    Vector3d euler(pose(3), pose(4), pose(5));
    Matrix3d R = eulerAnglesToRotationMatrix(euler);
    transform.block<3, 3>(0, 0) = R;
    
    return transform;
}

Vector3d rotationMatrixToEulerAngles(const Matrix3d& R) {
    Vector3d euler;
    
    if (abs(R(2, 0)) >= 1.0) {
        euler(1) = -M_PI/2 * R(2, 0);
        euler(0) = atan2(-R(0, 1), -R(0, 2));
        euler(2) = 0.0;
    } else {
        euler(1) = asin(-R(2, 0));
        euler(0) = atan2(R(2, 1)/cos(euler(1)), R(2, 2)/cos(euler(1)));
        euler(2) = atan2(R(1, 0)/cos(euler(1)), R(0, 0)/cos(euler(1)));
    }
    
    return euler;
}

Matrix3d eulerAnglesToRotationMatrix(const Vector3d& euler) {
    double alpha = euler(0);
    double beta = euler(1);
    double gamma = euler(2);
    
    Matrix3d R;
    R = AngleAxisd(alpha, Vector3d::UnitZ()) *
        AngleAxisd(beta, Vector3d::UnitY()) *
        AngleAxisd(gamma, Vector3d::UnitX());
    
    return R;
}

std::vector<DHParameters> generateRandomDHParameters(int num_links) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-M_PI, M_PI);
    std::uniform_int_distribution<int> type_dist(0, 2);
    
    std::vector<DHParameters> params;
    
    for (int i = 0; i < num_links; ++i) {
        DHParameters p;
        p.a = dist(gen);
        p.alpha = dist(gen);
        p.d = dist(gen);
        p.theta = dist(gen);
        
        int type = type_dist(gen);
        if (type == 0) {
            p.type = JointType::CONSTANT;
        } else if (type == 1) {
            p.type = JointType::REVOLUTE;
        } else {
            p.type = JointType::PRISMATIC;
        }
        
        params.push_back(p);
    }
    
    return params;
}

}
