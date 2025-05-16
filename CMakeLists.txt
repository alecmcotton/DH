cmake_minimum_required(VERSION 3.10)
project(kinematics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_library(kinematics STATIC src/kinematics.cpp)
add_executable(kinematics_test test/main.cpp)

target_link_libraries(kinematics_test kinematics Eigen3::Eigen)
