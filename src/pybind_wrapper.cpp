#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "dh_kinematics.h"

namespace py = pybind11;
using namespace dh_kinematics;

PYBIND11_MODULE(dh_kinematics, m) {
    py::enum_<JointType>(m, "JointType")
        .value("CONSTANT", JointType::CONSTANT)
        .value("REVOLUTE", JointType::REVOLUTE)
        .value("PRISMATIC", JointType::PRISMATIC);
    
    py::class_<DHParameters>(m, "DHParameters")
        .def_readwrite("a", &DHParameters::a)
        .def_readwrite("alpha", &DHParameters::alpha)
        .def_readwrite("d", &DHParameters::d)
        .def_readwrite("theta", &DHParameters::theta)
        .def_readwrite("type", &DHParameters::type);
    
    py::class_<DHKinematics>(m, "DHKinematics")
        .def(py::init<>())
        .def("load_from_csv", &DHKinematics::loadFromCSV)
        .def("forward", &DHKinematics::forward)
        .def("inverse", &DHKinematics::inverse)
        .def("jacobian", &DHKinematics::jacobian)
        .def("get_num_joints", &DHKinematics::getNumJoints)
        .def("get_dh_parameters", &DHKinematics::getDHParameters);
    
    m.def("generate_random_dh_parameters", &generateRandomDHParameters);
}
