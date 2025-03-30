#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include "rigidbody/RigidBodyState.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"
#include "collision/Geometry.h"
#include "joint/Joint.h"
#include "contact/Contact.h"
#include "util/MeshAssets.h"

namespace py = pybind11;

void bind_geometry(py::module_ &m) {
    // Base Geometry class
    py::class_<Geometry>(m, "Geometry", "Base class for all collision geometries")
        .def("computeInertia", &Geometry::computeInertia, 
             py::arg("mass"),
             "Compute the inertia tensor for this geometry with the given mass");
    
    // Bind derived geometry classes if available
    // For example:
    // py::class_<SphereGeometry, Geometry>(m, "SphereGeometry", "Sphere collision geometry")
    //     .def(py::init<float>(), py::arg("radius"), "Create a sphere with the given radius")
    //     .def_readwrite("radius", &SphereGeometry::radius, "Radius of the sphere");
    // 
    // py::class_<BoxGeometry, Geometry>(m, "BoxGeometry", "Box collision geometry")
    //     .def(py::init<const Eigen::Vector3f&>(), py::arg("halfExtents"), "Create a box with the given half-extents")
    //     .def_readwrite("halfExtents", &BoxGeometry::halfExtents, "Half-extents of the box");
}

void bind_mesh(py::module_ &m) {
    // Bind Mesh struct
    py::class_<Mesh>(m, "Mesh", "Triangular mesh data structure")
        .def(py::init<>(), "Default constructor")
        .def_readwrite("meshV", &Mesh::meshV, "Vertex positions")
        .def_readwrite("meshF", &Mesh::meshF, "Triangle face indices");
    
    // Bind MeshAssetRegistry if needed
    py::class_<MeshAssetRegistry>(m, "MeshAssetRegistry", "Registry for loaded mesh assets")
        .def_static("loadObj", &MeshAssetRegistry::loadObj, 
                   py::arg("filename"),
                   "Load an OBJ file and return the mesh, caching results for efficiency")
        .def_static("clearCache", &MeshAssetRegistry::clearCache,
                   "Clear the mesh cache");
}

void bind_contact(py::module_ &m) {
    // Bind Contact class
    py::class_<Contact>(m, "Contact", "Contact constraint between two rigid bodies")
        .def(py::init<RigidBody*, RigidBody*>(), 
             py::arg("body0"), py::arg("body1"),
             "Create a contact between two rigid bodies")
        .def_readwrite("body0", &Contact::body0, "First rigid body in the contact")
        .def_readwrite("body1", &Contact::body1, "Second rigid body in the contact")
        .def_property_readonly("lambda", [](const Contact& c) { return c.lambda; }, 
                              "Contact constraint force multiplier")
        .def_property_readonly("point", [](const Contact& c) { return c.point; }, 
                              "Contact point in world space");
}

void bind_joint(py::module_ &m) {
    // Bind Joint base class
    py::class_<Joint>(m, "Joint", "Base class for joints between rigid bodies")
        .def_readwrite("body0", &Joint::body0, "First rigid body in the joint")
        .def_readwrite("body1", &Joint::body1, "Second rigid body in the joint");
    
    // Add derived joint classes here when needed
    // For example:
    // py::class_<BallJoint, Joint>(m, "BallJoint", "Ball and socket joint")
    //     .def(py::init<RigidBody*, RigidBody*, const Eigen::Vector3f&>(),
    //          py::arg("body0"), py::arg("body1"), py::arg("worldPos"),
    //          "Create a ball joint at the given world position");
}

void bind_rigidbody(py::module_ &m) {
    // Bind RigidBody class
    py::class_<RigidBody>(m, "RigidBody", "Rigid body with physical properties and geometry")
        .def(py::init<float, Geometry*, const std::string&>(), 
             py::arg("mass"), py::arg("geometry"), py::arg("meshFilename") = "",
             "Create a rigid body with the given mass, geometry, and optional mesh file")
        .def(py::init<float, Geometry*, const Mesh&>(),
             py::arg("mass"), py::arg("geometry"), py::arg("mesh"),
             "Create a rigid body with the given mass, geometry, and mesh")
        .def("updateInertiaMatrix", &RigidBody::updateInertiaMatrix,
             "Update the world-space inertia matrix based on current orientation")
        .def("addForceAtPos", &RigidBody::addForceAtPos,
             py::arg("pos"), py::arg("force"),
             "Add a force at the specified world position")
        .def("getVelocityAtPos", &RigidBody::getVelocityAtPos,
             py::arg("pos"), py::arg("vel"),
             "Get velocity at the given world position")
        .def("getExtents", &RigidBody::getExtents,
             "Get the extents of the rigid body for bounding box visualization")
        .def_readwrite("fixed", &RigidBody::fixed,
             "Whether the body is static (fixed in place)")
        .def_readwrite("mass", &RigidBody::mass,
             "Mass of the rigid body")
        .def_readwrite("x", &RigidBody::x,
             "Position in world space")
        .def_readwrite("q", &RigidBody::q,
             "Orientation as a quaternion")
        .def_readwrite("xdot", &RigidBody::xdot,
             "Linear velocity")
        .def_readwrite("omega", &RigidBody::omega,
             "Angular velocity")
        .def_readwrite("f", &RigidBody::f,
             "Applied linear force")
        .def_readwrite("tau", &RigidBody::tau,
             "Applied torque")
        .def_readwrite("color", &RigidBody::color,
             "Color for visualization")
        .def_readwrite("name", &RigidBody::name,
             "Unique name for the body")
        .def_readwrite("texturePath", &RigidBody::texturePath,
             "Path to texture file")
        .def_readwrite("materialName", &RigidBody::materialName,
             "Name of material to apply")
        .def_property_readonly("contacts", [](const RigidBody& b) { return b.contacts; },
             "List of contacts involving this body")
        .def_property_readonly("joints", [](const RigidBody& b) { return b.joints; },
             "List of joints involving this body")
        .def_property_readonly("I", [](const RigidBody& b) { return b.I; },
             "World-space inertia tensor")
        .def_property_readonly("Iinv", [](const RigidBody& b) { return b.Iinv; },
             "World-space inverse inertia tensor")
        .def_property_readonly("Ibody", [](const RigidBody& b) { return b.Ibody; },
             "Body-space inertia tensor")
        .def_property_readonly("IbodyInv", [](const RigidBody& b) { return b.IbodyInv; },
             "Body-space inverse inertia tensor")
        .def_property_readonly("index", [](const RigidBody& b) { return b.index; },
             "Unique index for solver identification");
    
    // Add static variable
    m.attr("RigidBody").attr("counter") = &RigidBody::counter;
}

void bind_rigidbody_state(py::module_ &m) {
    // Bind RigidBodyState class
    py::class_<RigidBodyState>(m, "RigidBodyState", 
        "Class to store the kinematic state of a single rigid body")
        .def(py::init<>(), "Default constructor")
        .def(py::init<const RigidBody&>(), 
             py::arg("body"),
             "Construct from a RigidBody, automatically saving its state")
        .def("save", &RigidBodyState::save, 
             py::arg("body"),
             "Save the state from a RigidBody")
        .def("restore", &RigidBodyState::restore, 
             py::arg("body"),
             "Restore the saved state to a RigidBody");

    // Bind RigidBodySystemState class
    py::class_<RigidBodySystemState>(m, "RigidBodySystemState", 
        "Class to store the state of an entire rigid body system")
        .def(py::init<const RigidBodySystem&>(), 
             py::arg("system"),
             "Construct from a RigidBodySystem, automatically saving its state")
        .def("save", &RigidBodySystemState::save, 
             py::arg("system"),
             "Save the state from a RigidBodySystem")
        .def("restore", &RigidBodySystemState::restore, 
             py::arg("system"),
             "Restore the saved state to a RigidBodySystem");
}

void bind_rigidbody_system(py::module_ &m) {
    // Bind RigidBodySystem class
    py::class_<RigidBodySystem>(m, "RigidBodySystem", 
        "Class that manages and simulates a collection of rigid bodies and constraints")
        .def(py::init<>(), "Default constructor")
        .def("step", &RigidBodySystem::step, 
             py::arg("dt"),
             "Advance the simulation by the specified time step")
        .def("clear", &RigidBodySystem::clear,
             "Remove all rigid bodies and joints, and clean up memory")
        .def("addBody", &RigidBodySystem::addBody, 
             py::arg("body"),
             "Add a rigid body to the system (ownership is transferred to the system)")
        .def("addJoint", &RigidBodySystem::addJoint,
             py::arg("joint"),
             "Add a joint to the system (ownership is transferred to the system)")
        .def_property("bodies", 
            py::overload_cast<>(&RigidBodySystem::getBodies), 
            nullptr,
            "List of rigid bodies in the system")
        .def_property("contacts", 
            py::overload_cast<>(&RigidBodySystem::getContacts), 
            nullptr,
            "List of contacts in the system")
        .def_property("joints", 
            py::overload_cast<>(&RigidBodySystem::getJoints), 
            nullptr,
            "List of joints in the system")
        .def("setPreStepFunc", &RigidBodySystem::setPreStepFunc,
             py::arg("func"),
             "Set a callback function to be called before each simulation step")
        .def("setResetFunc", &RigidBodySystem::setResetFunc,
             py::arg("func"),
             "Set a callback function to be called when the system is reset")
        .def("setEnableCollisionDetection", &RigidBodySystem::setEnableCollisionDetection,
             py::arg("enableCollisions"),
             "Enable or disable collision detection")
        .def_readwrite("solverIter", &RigidBodySystem::solverIter,
             "Number of solver iterations per time step")
        .def_readwrite("solverId", &RigidBodySystem::solverId,
             "Solver type: 0=PGS, 1=Conjugate Gradient, 2=Conjugate Residual");
}

// This is the main binding module
PYBIND11_MODULE(physics_engine, m) {
    m.doc() = "Physics engine with rigid body dynamics and constraints";
    
    // Bind the classes
    bind_geometry(m);
    bind_mesh(m);
    bind_contact(m);
    bind_joint(m);
    bind_rigidbody(m);
    bind_rigidbody_state(m);
    bind_rigidbody_system(m);
    
    // Add the eigen utilities submodule
    py::module_ eigen = m.def_submodule("eigen", "Eigen-related utilities");
    
    // Add the scalar * quaternion operator
    eigen.def("multiply_scalar_quat", [](float a, const Eigen::Quaternionf& q) {
        return a * q;
    }, py::arg("scalar"), py::arg("quaternion"), "Multiply a scalar with a quaternion");
    
    // Add the quaternion + quaternion operator
    eigen.def("add_quat", [](const Eigen::Quaternionf& q1, const Eigen::Quaternionf& q2) {
        return q1 + q2;
    }, py::arg("q1"), py::arg("q2"), "Add two quaternions together");

    // Add common solver types as enum
    py::enum_<int>(m, "SolverType", "Types of constraint solvers")
        .value("PGS", 0, "Projected Gauss-Seidel solver")
        .value("CONJUGATE_GRADIENT", 1, "Conjugate Gradient solver")
        .value("CONJUGATE_RESIDUAL", 2, "Conjugate Residual solver")
        .export_values();
}