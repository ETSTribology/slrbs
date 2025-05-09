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
    PGSSM,
    PROXIMAL
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

    // Graph coloring toggle
    void setUseGraphColoring(bool enable) { m_useGraphColoring = enable; }
    bool getUseGraphColoring() const { return m_useGraphColoring; }

    // Gravity
    void setGravity(const Eigen::Vector3f& g) { m_gravity = g; }
    const Eigen::Vector3f& getGravity() const { return m_gravity; }

    // Solver settings
    void setSolverType(SolverType type);
    SolverType getSolverType() const;
    void setSolverIterations(int iters) { m_solverIter = iters; }
    int getSolverIterations() const { return m_solverIter; }

    // Integration method interface
    void setIntegrationMethod(IntegrationMethod method);
    IntegrationMethod getIntegrationMethod() const;
    void setIntegratorType(IntegratorType type) {
        setIntegrationMethod(static_cast<IntegrationMethod>(type));
    }
    IntegratorType getIntegratorType() const {
        return static_cast<IntegratorType>(getIntegrationMethod());
    }

    // Implicit-Euler parameter controls
    void setImplicitDamping(float d)       { m_implicitDamping = d; }
    float getImplicitDamping() const       { return m_implicitDamping; }

    void setGyroscopicDamping(float d)     { m_gyroDamping = d; }
    float getGyroscopicDamping() const     { return m_gyroDamping; }

    void setMaxLinearVelocity(float v)     { m_maxLinearVelocity = v; }
    float getMaxLinearVelocity() const     { return m_maxLinearVelocity; }

    void setMaxAngularVelocity(float v)    { m_maxAngularVelocity = v; }
    float getMaxAngularVelocity() const    { return m_maxAngularVelocity; }

    // Optionally disable velocity limiting entirely
    void setVelocityLimitingEnabled(bool e) { m_limitVelocities = e; }
    bool getVelocityLimitingEnabled() const { return m_limitVelocities; }

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
    bool m_collisionsEnabled = true;
    bool m_useGraphColoring = true;
    bool m_limitVelocities = true;
    PreStepFunc m_preStepFunc;
    ResetFunc m_resetFunc;
    Eigen::Vector3f m_gravity = {0.0f, -9.81f, 0.0f};
    SolverType m_solverType = SolverType::PGS;
    int m_solverIter = 10;
    IntegrationMethod m_integrationMethod = IntegrationMethod::EXPLICIT_EULER;

    // Implicit-Euler parameters
    float m_implicitDamping    = 0.98f;
    float m_gyroDamping        = 0.20f;
    float m_maxLinearVelocity  = 50.0f;
    float m_maxAngularVelocity = 20.0f;
};
