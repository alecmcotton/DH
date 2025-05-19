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
        .def("set_dh_parameters", &DHKinematics::setDHParameters)
        .def("forward", &DHKinematics::forward)
        .def("inverse", &DHKinematics::inverse)
        .def("jacobian", &DHKinematics::jacobian)
        .def("compute_manipulability", &DHKinematics::computeManipulability)
        .def("rotate_about_flange_x", &DHKinematics::rotateAboutFlangeX,  // 🔹 new
             py::arg("pose"), py::arg("zeta_degrees"))
        .def("get_num_actuated_joints", &DHKinematics::getNumActuatedJoints)
        .def("get_dh_parameters", &DHKinematics::getDHParameters)
        .def_static("compute_flange_pose_from_constraint", &DHKinematics::computeFlangePoseFromConstraint,
            py::arg("constraint_point"),
            py::arg("tool_tip_position"),
            py::arg("tool_length"),
            py::arg("tool_axis") = "z");

}
