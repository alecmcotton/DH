from setuptools import setup, Extension
from pybind11.setup_helpers import Pybind11Extension, build_ext
import os

# Use CONDA_PREFIX if set, otherwise default to /usr
eigen_path = os.path.join(os.getenv("CONDA_PREFIX", "/usr"), "include/eigen3")

ext_modules = [
    Pybind11Extension(
        "dh_kinematics",
        ["src/dh_kinematics.cpp", "src/pybind_wrapper.cpp"],
        include_dirs=["include", eigen_path],
        extra_compile_args=["-O3"],
    ),
]

setup(
    name="dh_kinematics",
    version="0.1",
    author="Your Name",
    description="DH Kinematics Library",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    python_requires=">=3.6",
)
