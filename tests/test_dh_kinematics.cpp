#include "../include/dh_kinematics.h"
#include <iostream>
#include <random>
#include <chrono>
#include <stdexcept>
#include <limits>

using namespace dh_kinematics;
using namespace std;
using namespace Eigen;

void runIKConvergenceTests() {
    const int NUM_TABLES = 100;
    const int TESTS_PER_TABLE = 100;
    const double TOLERANCE = 1e-6;
    
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<int> link_dist(3, 10);
    uniform_real_distribution<double> joint_dist(-M_PI, M_PI);
    
    int solvable_tests = 0;
    int solvable_passed = 0;
    int unsolvable_tests = 0;
    int unsolvable_caught = 0;
    
    auto start = chrono::high_resolution_clock::now();
    
    for (int i = 0; i < NUM_TABLES; ++i) {
        auto dh_table = generateRandomDHParameters(link_dist(gen));
        DHKinematics kin;
        kin.setDHParameters(dh_table);
        
        int num_joints = kin.getNumActuatedJoints();
        if (num_joints == 0) continue;
        
        for (int j = 0; j < TESTS_PER_TABLE; ++j) {
            solvable_tests++;
            
            VectorXd joints(num_joints);
            for (int k = 0; k < num_joints; ++k) {
                joints(k) = joint_dist(gen);
            }
            
            try {
                VectorXd target_pose = kin.forward(joints);
                VectorXd solved_joints = kin.inverse(target_pose, joints * 1.1, TOLERANCE);
                
                double final_error = (target_pose - kin.forward(solved_joints)).norm();
                if (final_error > TOLERANCE) {
                    cerr << "IK failed to reach tolerance (" << final_error << " > " << TOLERANCE << ")\n";
                } else {
                    solvable_passed++;
                }
            } catch (const exception& e) {
                cerr << "IK failed on solvable pose: " << e.what() << "\n";
            }
        }
    }
    
    for (int i = 0; i < NUM_TABLES/2; ++i) {
        auto dh_table = generateRandomDHParameters(link_dist(gen));
        DHKinematics kin;
        kin.setDHParameters(dh_table);
        
        int num_joints = kin.getNumActuatedJoints();
        if (num_joints == 0) continue;
        
        for (int j = 0; j < TESTS_PER_TABLE/10; ++j) {
            unsolvable_tests++;
            
            try {
                VectorXd random_pose(6);
                random_pose << uniform_real_distribution<double>(-10, 10)(gen),
                              uniform_real_distribution<double>(-10, 10)(gen),
                              uniform_real_distribution<double>(-10, 10)(gen),
                              uniform_real_distribution<double>(-M_PI, M_PI)(gen),
                              uniform_real_distribution<double>(-M_PI, M_PI)(gen),
                              uniform_real_distribution<double>(-M_PI, M_PI)(gen);
                
                VectorXd initial_guess = VectorXd::Zero(num_joints);
                VectorXd solved_joints = kin.inverse(random_pose, initial_guess, TOLERANCE);
                
                double error = (random_pose - kin.forward(solved_joints)).norm();
                cerr << "IK unexpectedly converged on unreachable pose (error: " << error << ")\n";
            } catch (const exception& e) {
                unsolvable_caught++;
            }
        }
    }
    
    auto end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);
    
    cout << "\nIK Validation Results:\n";
    cout << "=====================\n";
    cout << "Solvable poses: " << solvable_passed << "/" << solvable_tests 
         << " (" << 100.0*solvable_passed/solvable_tests << "%)\n";
    cout << "Unsolvable poses: " << unsolvable_caught << "/" << unsolvable_tests 
         << " (" << 100.0*unsolvable_caught/unsolvable_tests << "% detected)\n";
    cout << "Total time: " << duration.count() << " ms\n";
    
    if (solvable_passed == solvable_tests && 
        unsolvable_caught == unsolvable_tests) {
        cout << "\nALL TESTS PASSED!\n";
    } else {
        cout << "\nTEST FAILURES DETECTED!\n";
    }
}

int main() {
    try {
        runIKConvergenceTests();
        return 0;
    } catch (const exception& e) {
        cerr << "Test error: " << e.what() << endl;
        return 1;
    }
}
