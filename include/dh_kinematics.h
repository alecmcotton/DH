#pragma once
#include <vector>
#include <string>
#include <Eigen/Dense>

namespace dh_kinematics {

enum class JointType { CONSTANT, REVOLUTE, PRISMATIC };

struct DHParameters {
    double a;
    double alpha;
    double d;
    double theta;
    JointType type;
};

class DHKinematics {
public:
    DHKinematics() = default;
    
    bool loadFromCSV(const std::string& filename);
    void setDHParameters(const std::vector<DHParameters>& params);
    
    Eigen::VectorXd forward(const Eigen::VectorXd& joint_values) const;
    Eigen::VectorXd inverse(const Eigen::VectorXd& target_pose,
                          const Eigen::VectorXd& initial_guess,
                          double tolerance = 1e-6,
                          int max_iterations = 100) const;
    
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& joint_values,
                           double delta = 1e-6) const;
    
    int getNumActuatedJoints() const;
    const std::vector<DHParameters>& getDHParameters() const;

private:
    std::vector<DHParameters> dh_params_;
    
    Eigen::Matrix4d computeDHMatrix(double a, double alpha, double d, double theta) const;
    Eigen::VectorXd extractPose(const Eigen::Matrix4d& transform) const;
    Eigen::Matrix4d poseToTransform(const Eigen::VectorXd& pose) const;
};

Eigen::Vector3d rotationMatrixToEulerAngles(const Eigen::Matrix3d& R);
Eigen::Matrix3d eulerAnglesToRotationMatrix(const Eigen::Vector3d& euler);
std::vector<DHParameters> generateRandomDHParameters(int num_links);

}
