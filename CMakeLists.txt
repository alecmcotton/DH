cmake_minimum_required(VERSION 3.12)
project(dh_kinematics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Add this near the top of your CMakeLists.txt
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# Find Eigen
find_package(Eigen3 REQUIRED)

# Build main library
add_library(dh_kinematics STATIC
    src/dh_kinematics.cpp
)

target_include_directories(dh_kinematics PUBLIC 
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Build Python module
find_package(pybind11 REQUIRED)
pybind11_add_module(dh_kinematics_pybind src/pybind_wrapper.cpp)
target_link_libraries(dh_kinematics_pybind PRIVATE dh_kinematics)

# Build tests
add_executable(test_dh_kinematics tests/test_dh_kinematics.cpp)
target_link_libraries(test_dh_kinematics dh_kinematics)
