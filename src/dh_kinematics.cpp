#include "dh_kinematics.h"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <random>
#include <algorithm>

namespace dh_kinematics {

using namespace Eigen;

bool DHKinematics::loadFromCSV(const std::string& filename) {
    dh_params_.clear();
    std::ifstream file(filename);
    if (!file.is_open()) return false;
    
    std::string line;
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
        if (cell == "R") params.type = JointType::REVOLUTE;
        else if (cell == "P") params.type = JointType::PRISMATIC;
        else params.type = JointType::CONSTANT;
        
        dh_params_.push_back(params);
    }
    return true;
}

void DHKinematics::setDHParameters(const std::vector<DHParameters>& params) {
    dh_params_ = params;
}

int DHKinematics::getNumActuatedJoints() const {
    return std::count_if(dh_params_.begin(), dh_params_.end(),
        [](const DHParameters& p) { return p.type != JointType::CONSTANT; });
}

VectorXd DHKinematics::forward(const VectorXd& joint_values) const {
    if (joint_values.size() != getNumActuatedJoints()) {
        throw std::runtime_error("Invalid joint values size");
    }
    
    Matrix4d transform = Matrix4d::Identity();
    int joint_idx = 0;
    
    for (const auto& param : dh_params_) {
        double a = param.a;
        double alpha = param.alpha;
        double d = param.d;
        double theta = param.theta;
        
        if (param.type == JointType::REVOLUTE) {
            theta = joint_values[joint_idx++];
        } else if (param.type == JointType::PRISMATIC) {
            d = joint_values[joint_idx++];
        }
        
        transform *= computeDHMatrix(a, alpha, d, theta);
    }
    
    return extractPose(transform);
}

VectorXd DHKinematics::inverse(const VectorXd& target_pose,
                             const VectorXd& initial_guess,
                             double tolerance,
                             int max_iterations) const {
    VectorXd q = initial_guess;
    VectorXd prev_q;
    double prev_error = std::numeric_limits<double>::max();
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        VectorXd current_pose = forward(q);
        VectorXd error = target_pose - current_pose;
        double current_error = error.norm();
        
        if (current_error < tolerance) return q;
        if (current_error > prev_error * 1.5 && iter > 5) {
            throw std::runtime_error("IK is diverging");
        }
        if (iter > 10 && (q - prev_q).norm() < tolerance * 0.1) {
            throw std::runtime_error("IK is oscillating");
        }
        
        MatrixXd J = jacobian(q);
        q += J.jacobiSvd(ComputeThinU | ComputeThinV).solve(error);
        prev_q = q;
        prev_error = current_error;
    }
    
    throw std::runtime_error("IK failed to converge");
}

MatrixXd DHKinematics::jacobian(const VectorXd& joint_values, double delta) const {
    const int num_joints = getNumActuatedJoints();
    MatrixXd J(6, num_joints);
    VectorXd pose = forward(joint_values);
    
    for (int i = 0; i < num_joints; ++i) {
        VectorXd perturbed = joint_values;
        perturbed[i] += delta;
        VectorXd new_pose = forward(perturbed);
        J.col(i) = (new_pose - pose) / delta;
    }
    return J;
}

Matrix4d DHKinematics::computeDHMatrix(double a, double alpha, double d, double theta) const {
    Matrix4d mat;
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    double cos_alpha = cos(alpha);
    double sin_alpha = sin(alpha);
    
    mat << cos_theta, -sin_theta*cos_alpha,  sin_theta*sin_alpha, a*cos_theta,
           sin_theta,  cos_theta*cos_alpha, -cos_theta*sin_alpha, a*sin_theta,
           0,          sin_alpha,            cos_alpha,            d,
           0,          0,                    0,                    1;
    return mat;
}

VectorXd DHKinematics::extractPose(const Matrix4d& transform) const {
    VectorXd pose(6);
    pose(0) = transform(0,3);
    pose(1) = transform(1,3);
    pose(2) = transform(2,3);
    
    Matrix3d R = transform.block<3,3>(0,0);
    Vector3d euler = rotationMatrixToEulerAngles(R);
    pose(3) = euler(0);
    pose(4) = euler(1);
    pose(5) = euler(2);
    
    return pose;
}

Matrix4d DHKinematics::poseToTransform(const VectorXd& pose) const {
    Matrix4d transform = Matrix4d::Identity();
    transform(0,3) = pose(0);
    transform(1,3) = pose(1);
    transform(2,3) = pose(2);
    
    Vector3d euler(pose(3), pose(4), pose(5));
    transform.block<3,3>(0,0) = eulerAnglesToRotationMatrix(euler);
    return transform;
}

Vector3d rotationMatrixToEulerAngles(const Matrix3d& R) {
    Vector3d euler;
    if (abs(R(2,0)) >= 1.0) {
        euler(1) = -M_PI/2 * R(2,0);
        euler(0) = atan2(-R(0,1), -R(0,2));
        euler(2) = 0.0;
    } else {
        euler(1) = asin(-R(2,0));
        euler(0) = atan2(R(2,1)/cos(euler(1)), R(2,2)/cos(euler(1)));
        euler(2) = atan2(R(1,0)/cos(euler(1)), R(0,0)/cos(euler(1)));
    }
    return euler;
}

Matrix3d eulerAnglesToRotationMatrix(const Vector3d& euler) {
    Matrix3d R;
    R = AngleAxisd(euler(0), Vector3d::UnitZ()) *
        AngleAxisd(euler(1), Vector3d::UnitY()) *
        AngleAxisd(euler(2), Vector3d::UnitX());
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
        p.type = static_cast<JointType>(type_dist(gen));
        params.push_back(p);
    }
    return params;
}

Eigen::Matrix4d DHKinematics::computeFlangePoseFromConstraint(
    const Eigen::Vector3d& constraint_point,
    const Eigen::Vector3d& tool_tip_position,
    double tool_length,
    const std::string& tool_axis) 
{
    using namespace Eigen;
    
    if (tool_length <= 0) throw std::runtime_error("Tool length must be positive");
    Vector3d dir = (tool_tip_position - constraint_point).normalized();
    if (dir.norm() < 1e-6) throw std::runtime_error("Constraint and tip are coincident");

    Vector3d flange_position = tool_tip_position - tool_length * dir;
    Matrix3d R;

    if (tool_axis == "z") {
        Vector3d z_axis = dir;
        Vector3d x_axis = Vector3d::UnitZ().cross(z_axis).normalized();
        if (x_axis.norm() < 1e-6) x_axis = Vector3d::UnitX();
        Vector3d y_axis = z_axis.cross(x_axis).normalized();
        
        R.col(0) = x_axis;
        R.col(1) = y_axis;
        R.col(2) = z_axis;
    }
    else if (tool_axis == "x") {
        Vector3d x_axis = dir;
        Vector3d y_axis = x_axis.cross(Vector3d::UnitZ()).normalized();
        if (y_axis.norm() < 1e-6) y_axis = Vector3d::UnitY();
        Vector3d z_axis = x_axis.cross(y_axis).normalized();
        
        R.col(0) = x_axis;
        R.col(1) = y_axis;
        R.col(2) = z_axis;
    }
    else {
        throw std::runtime_error("Invalid tool axis, must be 'x' or 'z'");
    }

    Matrix4d flange_pose = Matrix4d::Identity();
    flange_pose.block<3,3>(0,0) = R;
    flange_pose.block<3,1>(0,3) = flange_position;
    return flange_pose;
}

double DHKinematics::computeManipulability(const Eigen::VectorXd& joint_values) const {
    Eigen::MatrixXd J = jacobian(joint_values);
    Eigen::MatrixXd JJt = J * J.transpose();
    double det = JJt.determinant();

    if (det < 0) return 0.0;

    return std::sqrt(det);
}

Eigen::VectorXd dh_kinematics::DHKinematics::rotateAboutFlangeX(const Eigen::VectorXd& pose, double zeta_degrees) const {
    assert(pose.size() == 6 && "Pose must be of size 6: x, y, z, alpha, beta, gamma");

    Eigen::Vector3d position = pose.head<3>();
    Eigen::Vector3d euler_angles = pose.tail<3>();  // Assuming XYZ convention (alpha, beta, gamma)

    // Convert Euler angles to rotation matrix
    Eigen::Matrix3d R = eulerAnglesToRotationMatrix(euler_angles);

    // Create rotation around the local X-axis (flange frame)
    double zeta_rad = zeta_degrees * M_PI / 180.0;
    Eigen::Matrix3d R_zeta = Eigen::AngleAxisd(zeta_rad, Eigen::Vector3d::UnitX()).toRotationMatrix();

    // Apply the rotation in the local frame
    Eigen::Matrix3d R_new = R * R_zeta;

    // Convert back to Euler angles
    Eigen::Vector3d new_euler_angles = rotationMatrixToEulerAngles(R_new);

    // Combine with original position
    Eigen::VectorXd new_pose(6);
    new_pose.head<3>() = position;
    new_pose.tail<3>() = new_euler_angles;

    return new_pose;
}

}
