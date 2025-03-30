#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "joint/Joint.h"
#include "joint/Distance.h"
#include "joint/Hinge.h"
#include "joint/Prismatic.h"
#include "joint/Spherical.h"
#include "joint/Universal.h"
#include "contact/Contact.h"

namespace py = pybind11;

// Forward declarations of binding functions
void bind_joint(py::module_ &m);
void bind_contact(py::module_ &m);
void bind_distance(py::module_ &m);
void bind_hinge(py::module_ &m);
void bind_prismatic(py::module_ &m);
void bind_spherical(py::module_ &m);
void bind_universal(py::module_ &m);

// Bind Joint base class
void bind_joint(py::module_ &m) {
    py::class_<Joint, std::shared_ptr<Joint>>(m, "Joint")
        .def(py::init<RigidBody*, RigidBody*, const Eigen::Vector3f&, const Eigen::Quaternionf&, const Eigen::Vector3f&, const Eigen::Quaternionf&>(),
            py::arg("body0"), py::arg("body1"), 
            py::arg("r0"), py::arg("q0"),
            py::arg("r1"), py::arg("q1"))
        .def(py::init<RigidBody*, RigidBody*>(),
            py::arg("body0"), py::arg("body1"))
        .def_readwrite("body0", &Joint::body0)
        .def_readwrite("body1", &Joint::body1)
        .def_readwrite("J0", &Joint::J0)
        .def_readwrite("J1", &Joint::J1)
        .def_readwrite("J0Minv", &Joint::J0Minv)
        .def_readwrite("J1Minv", &Joint::J1Minv)
        .def_readwrite("phi", &Joint::phi)
        .def_readwrite("lambda", &Joint::lambda)
        .def_readwrite("idx", &Joint::idx)
        .def_readwrite("dim", &Joint::dim)
        .def_readwrite("r0", &Joint::r0)
        .def_readwrite("r1", &Joint::r1)
        .def_readwrite("q0", &Joint::q0)
        .def_readwrite("q1", &Joint::q1)
        .def("getType", &Joint::getType)
        .def("computeJacobian", &Joint::computeJacobian);
}

// Bind Contact class
void bind_contact(py::module_ &m) {
    py::class_<Contact, Joint, std::shared_ptr<Contact>>(m, "Contact")
        .def(py::init<RigidBody*, RigidBody*, const Eigen::Vector3f&, const Eigen::Vector3f&, float>(),
             py::arg("body0"), py::arg("body1"), py::arg("p"), py::arg("n"), py::arg("pene"))
        .def_readwrite_static("mu", &Contact::mu)
        .def_readwrite("p", &Contact::p)
        .def_readwrite("n", &Contact::n)
        .def_readwrite("t", &Contact::t)
        .def_readwrite("b", &Contact::b)
        .def_readwrite("pene", &Contact::pene)
        .def("computeJacobian", &Contact::computeJacobian)
        .def("computeContactFrame", &Contact::computeContactFrame)
        .def("getType", &Contact::getType);
}

// Bind Distance joint class
void bind_distance(py::module_ &m) {
    py::class_<Distance, Joint, std::shared_ptr<Distance>>(m, "Distance")
        .def(py::init<RigidBody*, RigidBody*, const Eigen::Vector3f&, const Eigen::Vector3f&, float>(),
             py::arg("body0"), py::arg("body1"), py::arg("r0"), py::arg("r1"), py::arg("d"))
        .def_readwrite("d", &Distance::d)
        .def("computeJacobian", &Distance::computeJacobian);
}

// Bind Hinge joint class
void bind_hinge(py::module_ &m) {
    py::class_<Hinge, Joint, std::shared_ptr<Hinge>>(m, "Hinge")
        .def(py::init<RigidBody*, RigidBody*, const Eigen::Vector3f&, const Eigen::Quaternionf&, const Eigen::Vector3f&, const Eigen::Quaternionf&>(),
             py::arg("body0"), py::arg("body1"), 
             py::arg("r0"), py::arg("q0"), 
             py::arg("r1"), py::arg("q1"))
        .def("computeJacobian", &Hinge::computeJacobian)
        .def("getType", &Hinge::getType);
}

// Bind Prismatic joint class
void bind_prismatic(py::module_ &m) {
    py::class_<Prismatic, Joint, std::shared_ptr<Prismatic>>(m, "Prismatic")
        .def(py::init<RigidBody*, RigidBody*, const Eigen::Vector3f&, const Eigen::Vector3f&, const Eigen::Vector3f&>(),
             py::arg("body0"), py::arg("body1"), 
             py::arg("r0"), py::arg("r1"), 
             py::arg("axis"))
        .def("computeJacobian", &Prismatic::computeJacobian);
}

// Bind Spherical joint class
void bind_spherical(py::module_ &m) {
    py::class_<Spherical, Joint, std::shared_ptr<Spherical>>(m, "Spherical")
        .def(py::init<RigidBody*, RigidBody*, const Eigen::Vector3f&, const Eigen::Vector3f&>(),
             py::arg("body0"), py::arg("body1"), 
             py::arg("r0"), py::arg("r1"))
        .def("computeJacobian", &Spherical::computeJacobian)
        .def("getType", &Spherical::getType);
}

// Bind Universal joint class
void bind_universal(py::module_ &m) {
    py::class_<Universal, Joint, std::shared_ptr<Universal>>(m, "Universal")
        .def(py::init<RigidBody*, RigidBody*, const Eigen::Vector3f&, const Eigen::Quaternionf&, const Eigen::Vector3f&, const Eigen::Quaternionf&>(),
             py::arg("body0"), py::arg("body1"), 
             py::arg("r0"), py::arg("q0"), 
             py::arg("r1"), py::arg("q1"))
        .def("computeJacobian", &Universal::computeJacobian);
}

// Main module definition just for joints
PYBIND11_MODULE(physics_joints, m) {
    m.doc() = "Physics engine joint system";
    
    // Define the constraint type enum
    py::enum_<eConstraintType>(m, "ConstraintType")
        .value("Contact", eConstraintType::kContact)
        .value("Spherical", eConstraintType::kSpherical)
        .value("Hinge", eConstraintType::kHinge)
        .export_values();
    
    // Bind all joint classes
    bind_joint(m);
    bind_contact(m);
    bind_distance(m);
    bind_hinge(m);
    bind_prismatic(m);
    bind_spherical(m);
    bind_universal(m);
}