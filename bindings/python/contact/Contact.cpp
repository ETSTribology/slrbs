#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "Contact.h"

namespace py = pybind11;

void bind_contact(py::module_ &m) {
    // Bind the Contact class with detailed docstrings and proper parameter naming
    py::class_<Contact, Joint, std::shared_ptr<Contact>>(m, "Contact", 
        "Contact constraint with box friction model.\n"
        "Stores contact normal, point and computed Jacobians for both bodies.")
        .def(py::init<RigidBody*, RigidBody*, const Eigen::Vector3f&, const Eigen::Vector3f&, float>(),
             py::arg("body0"), py::arg("body1"), py::arg("p"), py::arg("n"), py::arg("pene"),
             "Create a contact constraint between two bodies.\n\n"
             "Args:\n"
             "    body0: First rigid body in contact\n"
             "    body1: Second rigid body in contact\n"
             "    p: Contact point in world coordinates\n"
             "    n: Contact normal pointing from body1 to body0\n"
             "    pene: Penetration depth (positive for overlapping bodies)")
        .def_property_readonly_static("mu", [](py::object) { return Contact::mu; },
             "Global coefficient of friction for all contacts")
        .def_property_static("mu", 
             [](py::object) { return Contact::mu; },
             [](py::object, float value) { Contact::mu = value; },
             "Global coefficient of friction for all contacts")
        .def_readwrite("p", &Contact::p, "Contact point in world coordinates")
        .def_readwrite("n", &Contact::n, "Contact normal in world coordinates")
        .def_readwrite("t", &Contact::t, "First tangent direction (forms a basis with n and b)")
        .def_readwrite("b", &Contact::b, "Second tangent direction (forms a basis with n and t)")
        .def_readwrite("pene", &Contact::pene, "Penetration depth")
        .def("computeJacobian", &Contact::computeJacobian,
             "Compute the contact Jacobian matrices J0 and J1 for the constraint solver.\n"
             "This updates the J0, J1, J0Minv, and J1Minv matrices.")
        .def("computeContactFrame", &Contact::computeContactFrame,
             "Compute the contact coordinate frame (n, t, b) where n is the contact normal\n"
             "and t, b are tangent directions for friction calculations.")
        .def("getType", &Contact::getType,
             "Returns the constraint type (always kContact).");
}

// This can be included in your main binding file:
PYBIND11_MODULE(physics_engine, m) {
    m.doc() = "Physics engine with constraint-based dynamics and contact simulation";
    
    // Create physics submodule
    py::module_ physics = m.def_submodule("physics", "Physics engine core components");
    
    // Bind the eConstraintType enum
    py::enum_<eConstraintType>(physics, "ConstraintType", "Types of constraints in the system")
        .value("Contact", eConstraintType::kContact, "Contact constraint with friction")
        .value("Spherical", eConstraintType::kSpherical, "Spherical (ball-and-socket) joint")
        .value("Hinge", eConstraintType::kHinge, "Hinge (revolute) joint")
        .export_values();
    
    // Bind only the Contact class
    bind_contact(physics);
}