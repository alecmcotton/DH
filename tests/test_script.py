import numpy as np
import dh_kinematics
import random
import time

def run_python_tests():
    NUM_DH_TABLES = 100
    NUM_TESTS_PER_TABLE = 100
    TOLERANCE = 1e-4
    
    total_tests = 0
    passed_tests = 0
    
    start_time = time.time()
    
    for i in range(NUM_DH_TABLES):
        num_links = random.randint(1, 10)
        dh_params = dh_kinematics.generate_random_dh_parameters(num_links)
        
        kin = dh_kinematics.DHKinematics()
        kin.get_dh_parameters().extend(dh_params)
        
        num_joints = kin.get_num_joints()
        if num_joints == 0:
            continue  # this is shady
            
        for j in range(NUM_TESTS_PER_TABLE):
            joint_values = np.random.uniform(-np.pi, np.pi, num_joints)
            pose = kin.forward(joint_values)
            
            solved_joints = kin.inverse(pose, joint_values, TOLERANCE)
            solved_pose = kin.forward(solved_joints)
            
            error = np.linalg.norm(pose - solved_pose)
            
            total_tests += 1
            if error < TOLERANCE:
                passed_tests += 1
            else:
                print(f"Test failed with error: {error}")
    
    end_time = time.time()
    duration = end_time - start_time
    
    print("\nTest results:")
    print(f"Total tests: {total_tests}")
    print(f"Passed tests: {passed_tests} ({100 * passed_tests / total_tests:.2f}%)")
    print(f"Execution time: {duration:.2f} seconds")
    
    if passed_tests == total_tests:
        print("ALL TESTS PASSED!")
    else:
        print("SOME TESTS FAILED!")

if __name__ == "__main__":
    run_python_tests()
