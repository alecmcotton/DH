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
    
    // Load DH parameters from CSV file
    bool loadFromCSV(const std::string& filename);
    
    // Forward kinematics: joint values -> pose (x,y,z, alpha,beta,gamma)
    Eigen::VectorXd forward(const Eigen::VectorXd& joint_values) const;
    
    // Inverse kinematics: pose -> joint values (using Newton-Raphson)
    Eigen::VectorXd inverse(const Eigen::VectorXd& target_pose, 
                           const Eigen::VectorXd& initial_guess,
                           double tolerance = 1e-6,
                           int max_iterations = 100) const;
    
    // Compute Jacobian matrix numerically
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& joint_values, 
                            double delta = 1e-6) const;
    
    // Get number of actuated joints
    int getNumJoints() const;
    
    // Get all DH parameters
    const std::vector<DHParameters>& getDHParameters() const;
    void setDHParameters(const std::vector<DHParameters>& params);

private:
    std::vector<DHParameters> dh_params_;
    std::vector<int> actuated_joint_indices_;
    
    // Helper functions
    Eigen::Matrix4d computeDHMatrix(double a, double alpha, double d, double theta) const;
    Eigen::VectorXd extractPose(const Eigen::Matrix4d& transform) const;
    Eigen::Matrix4d poseToTransform(const Eigen::VectorXd& pose) const;
    void updateDHParameters(Eigen::VectorXd& joint_values) const;
};

// Utility function for testing
std::vector<DHParameters> generateRandomDHParameters(int num_links);
}
