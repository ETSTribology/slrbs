#pragma once

#include <memory>
#include <vector>
#include <functional>
#include <Eigen/Core>
#include <Eigen/Geometry>

class Contact;
class CollisionDetect;
class Joint;
class Solver;
class RigidBody;

typedef std::function<void(std::vector<RigidBody*>&)> PreStepFunc;
typedef std::function<void()> ResetFunc;

// Integration methods for advancing the system state
enum class IntegrationMethod {
    EXPLICIT_EULER,
    SYMPLECTIC_EULER,
    VERLET,
    RK4,
    IMPLICIT_EULER
};

// UI-compatible enum that maps to IntegrationMethod
enum class IntegratorType {
    EXPLICIT_EULER = 0,
    SYMPLECTIC_EULER = 1,
    VERLET = 2,
    RK4 = 3,
    IMPLICIT_EULER = 4
};

// Solver types for constraint solving
enum class SolverType {
    PGS,
    CONJ_GRADIENT,
    CONJ_RESIDUAL,
    PGSSM
};

class RigidBodySystem
{
public:
    // Constructor/destructor
    RigidBodySystem();
    virtual ~RigidBodySystem();

    // Advance the simulation by dt
    void step(float dt);

    // Remove all bodies/joints and reset
    void clear();

    // Add a rigid body (system takes ownership)
    void addBody(RigidBody* b);

    // Add a joint (system takes ownership)
    void addJoint(Joint* j);

    // Accessors
    const std::vector<RigidBody*>& getBodies() const { return m_bodies; }
    std::vector<RigidBody*>& getBodies() { return m_bodies; }
    const std::vector<Contact*>& getContacts() const;
    std::vector<Contact*>& getContacts();
    const std::vector<Joint*>& getJoints() const { return m_joints; }
    std::vector<Joint*>& getJoints() { return m_joints; }

    // Callbacks
    void setPreStepFunc(PreStepFunc func) { m_preStepFunc = func; }
    void setResetFunc(ResetFunc func) { m_resetFunc = func; }

    // Collision toggle
    void setEnableCollisionDetection(bool enable) { m_collisionsEnabled = enable; }
    bool getEnableCollisionDetection() const { return m_collisionsEnabled; }

    // Gravity
    void setGravity(const Eigen::Vector3f& g) { m_gravity = g; }
    const Eigen::Vector3f& getGravity() const { return m_gravity; }

    // Solver settings
    void setSolverType(SolverType type);
    SolverType getSolverType() const;
    void setSolverIterations(int iters) { m_solverIter = iters; }
    int getSolverIterations() const { return m_solverIter; }

    // Original integration method interface
    void setIntegrationMethod(IntegrationMethod method);
    IntegrationMethod getIntegrationMethod() const;

    // UI-compatible integration method interface
    void setIntegratorType(IntegratorType type) {
        // Map from UI enum to internal enum
        setIntegrationMethod(static_cast<IntegrationMethod>(type));
    }

    IntegratorType getIntegratorType() const {
        // Map from internal enum to UI enum
        return static_cast<IntegratorType>(getIntegrationMethod());
    }

private:
    // Internal pipelines
    void computeInertias();
    void calcConstraintForces(float dt);

    // Integration routines
    void integrateExplicitEuler(float dt);
    void integrateSymplecticEuler(float dt);
    void integrateVerlet(float dt);
    void integrateRK4(float dt);
    void integrateImplicitEuler(float dt);

    // Members
    std::vector<RigidBody*> m_bodies;
    std::vector<Joint*> m_joints;
    std::unique_ptr<CollisionDetect> m_collisionDetect;
    bool m_collisionsEnabled;
    PreStepFunc m_preStepFunc;
    ResetFunc m_resetFunc;
    Eigen::Vector3f m_gravity;
    SolverType m_solverType;
    int m_solverIter;
    IntegrationMethod m_integrationMethod;
};