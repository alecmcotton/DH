# DH Kinematics Library

A C++/Python library for robotic manipulator kinematics using Denavit-Hartenberg (DH) parameters with pybind11 Python bindings.

## Features
- **Complete DH Implementation**:
  - Forward and inverse kinematics
  - Jacobian computation
  - Singularity detection via manipulability measure
  - Support for revolute, prismatic, and fixed joints
- **Python Bindings**:
  - Full Python interface via pybind11
  - Numpy array compatibility
  - Clean Pythonic API
- **Advanced Functionality**:
  - Tool flange pose computation
  - Local frame rotations
  - Random robot generation

## Installation
### Prerequisites
- Eigen3 (≥3.3)
- pybind11
- C++17 compiler
- Python 3.6+

### Build Instructions
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
