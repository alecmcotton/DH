cmake_minimum_required(VERSION 3.12)
project(dh_kinematics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Dependencies
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)

# C++ static library
add_library(dh_kinematics_lib STATIC
    src/dh_kinematics.cpp
)
target_include_directories(dh_kinematics_lib PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Python bindings (module)
pybind11_add_module(dh_kinematics_pybind
    src/pybind_wrapper.cpp
)
target_link_libraries(dh_kinematics_pybind PRIVATE dh_kinematics_lib)

# Unit test executable
add_executable(test_dh_kinematics
    tests/test_dh_kinematics.cpp
)
target_link_libraries(test_dh_kinematics PRIVATE dh_kinematics_lib)
