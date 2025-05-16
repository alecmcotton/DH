cmake_minimum_required(VERSION 3.12)
project(dh_kinematics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)

add_library(dh_kinematics STATIC
    src/dh_kinematics.cpp
)
target_include_directories(dh_kinematics PUBLIC 
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

pybind11_add_module(dh_kinematics_pybind src/pybind_wrapper.cpp)
target_link_libraries(dh_kinematics_pybind PRIVATE dh_kinematics)

add_executable(test_dh_kinematics tests/test_dh_kinematics.cpp)
target_link_libraries(test_dh_kinematics dh_kinematics)
