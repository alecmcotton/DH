#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "dh_kinematics.h"

namespace py = pybind11;
using namespace dh_kinematics;

PYBIND11_MODULE(dh_kinematics, m) {
    py::enum_<dh_kinematics::JointType>(m, "JointType")
        .value("CONSTANT", dh_kinematics::JointType::CONSTANT)
        .value("REVOLUTE", dh_kinematics::JointType::REVOLUTE)
        .value("PRISMATIC", dh_kinematics::JointType::PRISMATIC);
    
    py::class_<dh_kinematics::DHParameters>(m, "DHParameters")
        .def_readwrite("a", &dh_kinematics::DHParameters::a)
        .def_readwrite("alpha", &dh_kinematics::DHParameters::alpha)
        .def_readwrite("d", &dh_kinematics::DHParameters::d)
        .def_readwrite("theta", &dh_kinematics::DHParameters::theta)
        .def_readwrite("type", &dh_kinematics::DHParameters::type);
    
    py::class_<dh_kinematics::DHKinematics>(m, "DHKinematics")
        .def(py::init<>())
        .def("load_from_csv", &dh_kinematics::DHKinematics::loadFromCSV)
        .def("set_dh_parameters", &dh_kinematics::DHKinematics::setDHParameters)
        .def("forward", &dh_kinematics::DHKinematics::forward)
        .def("inverse", &dh_kinematics::DHKinematics::inverse)
        .def("jacobian", &dh_kinematics::DHKinematics::jacobian)
        .def("get_num_actuated_joints", &dh_kinematics::DHKinematics::getNumActuatedJoints)
        .def("get_dh_parameters", &dh_kinematics::DHKinematics::getDHParameters);
    
    m.def("generate_random_dh_parameters", &dh_kinematics::generateRandomDHParameters);
}
