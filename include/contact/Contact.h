#pragma once

#include "joint/Joint.h"
#include <Eigen/Dense>

// Forward declarations
class RigidBody;

class Contact : public Joint
{
public:
    Contact();
    Contact(RigidBody* body0, RigidBody* body1, const Eigen::Vector3f& p, const Eigen::Vector3f& n, float pene);
    virtual ~Contact();

    virtual void computeJacobian() override;
    virtual eConstraintType getType() const override { return kContact; }

    // Warmstart the contact with previous solution
    void warmStart();

    // Reset contact accumulators
    void reset();

    void computeContactFrame();

    // Contact parameters
    Eigen::Vector3f p;     // Contact point
    Eigen::Vector3f n;     // Contact normal
    Eigen::Vector3f t;     // First tangent direction
    Eigen::Vector3f b;     // Second tangent direction (bitangent)
    float pene;            // Penetration depth

    // Relative velocity at contact point
    Eigen::Vector3f relVel;

    // Contact properties
    float restitution;     // Coefficient of restitution (bounciness)
    float bias;            // Baumgarte stabilization term
    bool persistent;       // Whether this contact persisted from last frame

    // Previous frame lambda for warmstarting
    Eigen::Vector3f prevLambda;

    // Friction coefficient
    static float mu;
    // Restitution threshold - minimum normal velocity for bounce
    static float restitutionThreshold;
    // Baumgarte stabilization factor
    static float baumgarte;
    // Slop factor for penetration depth
    static float slop;
    float k;  // constraint spring stiffness (mixing)

    using JBlock          = Eigen::Matrix<float,3,6>;
    using JBlockTranspose = Eigen::Matrix<float,6,3>;
    JBlockTranspose MinvJ0T, MinvJ1T;
};