#include "../include/dh_kinematics.h"
#include <iostream>
#include <random>
#include <chrono>

using namespace dh_kinematics;
using namespace std;
using namespace Eigen;

void runTests() {
    const int NUM_DH_TABLES = 100;
    const int NUM_TESTS_PER_TABLE = 100;
    const double TOLERANCE = 1e-4;
    
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<int> link_dist(1, 10);
    
    int total_tests = 0;
    int passed_tests = 0;
    
    auto start = chrono::high_resolution_clock::now();
    
    for (int i = 0; i < NUM_DH_TABLES; ++i) {
        int num_links = link_dist(gen);
        vector<DHParameters> dh_params = generateRandomDHParameters(num_links);
        
        DHKinematics kin;
        kin.setDHParameters(dh_params);
        
        int num_joints = kin.getNumJoints();
        if (num_joints == 0) continue; // Skip if no actuated joints
        
        uniform_real_distribution<double> joint_dist(-M_PI, M_PI);
        
        for (int j = 0; j < NUM_TESTS_PER_TABLE; ++j) {
            VectorXd joint_values(num_joints);
            for (int k = 0; k < num_joints; ++k) {
                joint_values(k) = joint_dist(gen);
            }
            
            VectorXd pose = kin.forward(joint_values);
            VectorXd solved_joints = kin.inverse(pose, joint_values, TOLERANCE);
            
            VectorXd solved_pose = kin.forward(solved_joints);
            double error = (pose - solved_pose).norm();
            
            total_tests++;
            if (error < TOLERANCE) {
                passed_tests++;
            } else {
                cout << "Test failed with error: " << error << endl;
            }
        }
    }
    
    auto end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);
    
    cout << "Test results:" << endl;
    cout << "Total tests: " << total_tests << endl;
    cout << "Passed tests: " << passed_tests << " (" 
         << (100.0 * passed_tests / total_tests) << "%)" << endl;
    cout << "Execution time: " << duration.count() << " ms" << endl;
    
    if (passed_tests == total_tests) {
        cout << "ALL TESTS PASSED!" << endl;
    } else {
        cout << "SOME TESTS FAILED!" << endl;
    }
}

int main() {
    runTests();
    return 0;
}
