#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "friction/FrictionModel.h"  // Added new include
#include <algorithm>
#include <cmath>

// Initialize static members with default values
float Contact::defaultStaticFriction = 0.8f;
float Contact::defaultKineticFriction = 0.3f;
float Contact::frictionVelocityThreshold = 0.1f;  // Velocity threshold for static/kinetic transition

Contact::Contact() : Joint(), p(), n(), t(), b(), pene(0.0f),
                   staticFriction(defaultStaticFriction), kineticFriction(defaultKineticFriction)
{
}

Contact::Contact(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _p, const Eigen::Vector3f& _n, float _pene) :
    Joint(_body0, _body1),
    p(_p), 
    n(_n.normalized()), // Ensure normal is normalized
    t(), 
    b(), 
    pene(_pene),
    staticFriction(defaultStaticFriction),
    kineticFriction(defaultKineticFriction)
{
    dim = 3; // 1 normal + 2 tangent directions for friction
    J0.setZero(3, 6);
    J1.setZero(3, 6);
    J0Minv.setZero(3, 6);
    J1Minv.setZero(3, 6);
    lambda.setZero(3);
    phi.setZero(3);
    phi(0) = _pene;  // Penetration depth in normal direction
    
    // Register contact with both bodies
    registerWithBodies();
    
    // Compute the contact frame
    computeContactFrame();
    
    // Update friction coefficients based on material properties
    if (body0 && body1 && body0->material.length() > 0 && body1->material.length() > 0) {
        updateFrictionCoefficients(body0->material, body1->material);
    }
}

Contact::~Contact()
{
    // Contact cleanup can go here if needed
}

void Contact::registerWithBodies()
{
    if (body0) {
        body0->contacts.push_back(this);
    }
    
    if (body1) {
        body1->contacts.push_back(this);
    }
}

void Contact::computeContactFrame()
{
    // Compute the contact frame, which consists of an orthonormal
    // basis formed by the vectors n, t, and b
    
    // Find first tangent direction
    if (std::abs(n.x()) > 0.9f) {
        // If normal is close to x-axis, use y-axis to find perpendicular
        t = n.cross(Eigen::Vector3f(0.0f, 1.0f, 0.0f));
    } else {
        // Otherwise use x-axis
        t = n.cross(Eigen::Vector3f(1.0f, 0.0f, 0.0f));
    }
    
    // Normalize first tangent
    float tLength = t.norm();
    if (tLength < 1e-6f) {
        // Fallback if cross product is too small
        t = n.cross(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
        tLength = t.norm();
    }
    t /= tLength;
    
    // Second tangent is perpendicular to both normal and first tangent
    b = n.cross(t).normalized();
}

void Contact::computeJacobian()
{
    // Compute the Jacobians J0 and J1
    const Eigen::Vector3f r0 = p - body0->x;  // Vector from body0 center to contact point
    const Eigen::Vector3f r1 = p - body1->x;  // Vector from body1 center to contact point
    
    // Reset all matrices
    J0.setZero(3, 6);
    J1.setZero(3, 6);
    J0Minv.setZero(3, 6);
    J1Minv.setZero(3, 6);
    lambda.setZero(3);
    phi.setZero(3);
    phi(0) = pene;  // Update penetration
    
    // Row 0: Normal direction (non-penetration constraint)
    // Format for each row: [linear part (3) | angular part (3)]
    
    // Normal row (body0)
    J0.block(0, 0, 1, 3) = -n.transpose();  // Linear velocity contribution
    J0.block(0, 3, 1, 3) = -r0.cross(n).transpose();  // Angular velocity contribution
    
    // Normal row (body1)
    J1.block(0, 0, 1, 3) = n.transpose();
    J1.block(0, 3, 1, 3) = r1.cross(n).transpose();
    
    // Tangent direction 1 (friction in first tangent direction)
    J0.block(1, 0, 1, 3) = -t.transpose();
    J0.block(1, 3, 1, 3) = -r0.cross(t).transpose();
    J1.block(1, 0, 1, 3) = t.transpose();
    J1.block(1, 3, 1, 3) = r1.cross(t).transpose();
    
    // Tangent direction 2 (friction in second tangent direction)
    J0.block(2, 0, 1, 3) = -b.transpose();
    J0.block(2, 3, 1, 3) = -r0.cross(b).transpose();
    J1.block(2, 0, 1, 3) = b.transpose();
    J1.block(2, 3, 1, 3) = r1.cross(b).transpose();
    
    // Compute the mass-weighted Jacobian blocks (J*M^-1) for each body
    // These are used in the constraint solver
    if (!body0->fixed) {
        J0Minv.block(0, 0, 3, 3) = J0.block(0, 0, 3, 3) * (1.0f / body0->mass);
        J0Minv.block(0, 3, 3, 3) = J0.block(0, 3, 3, 3) * body0->Iinv;
    }
    
    if (!body1->fixed) {
        J1Minv.block(0, 0, 3, 3) = J1.block(0, 0, 3, 3) * (1.0f / body1->mass);
        J1Minv.block(0, 3, 3, 3) = J1.block(0, 3, 3, 3) * body1->Iinv;
    }
}

float Contact::calculateRelativeTangentialVelocity() const
{
    if (!body0 || !body1) {
        return 0.0f;
    }
    
    // Calculate relative velocity at contact point
    Eigen::Vector3f v0 = body0->v + body0->omega.cross(p - body0->x);
    Eigen::Vector3f v1 = body1->v + body1->omega.cross(p - body1->x);
    Eigen::Vector3f vRel = v1 - v0;
    
    // Project onto tangent plane (remove normal component)
    Eigen::Vector3f vRelTangent = vRel - vRel.dot(n) * n;
    
    // Return magnitude of tangential velocity
    return vRelTangent.norm();
}

float Contact::getSmoothFrictionCoefficient(float relTangentialVelocity) const {
    // Now call the separated friction model function.
    return friction::getSmoothFrictionCoefficient(relTangentialVelocity,
                                                  defaultStaticFriction,
                                                  defaultKineticFriction,
                                                  frictionVelocityThreshold);
}

void Contact::updateFrictionCoefficients(const std::string& materialName0, const std::string& materialName1)
{
    // Try to get friction coefficients from the material pair registry
    float static_mu, kinetic_mu;
    
    if (FrictionRegistry::getInstance().getFrictionCoefficients(materialName0, materialName1, static_mu, kinetic_mu)) {
        // Materials found in registry
        staticFriction = static_mu;
        kineticFriction = kinetic_mu;
    } else {
        // Fall back to defaults
        staticFriction = defaultStaticFriction;
        kineticFriction = defaultKineticFriction;
    }
    
    // Ensure kinetic friction is not greater than static friction
    kineticFriction = std::min(kineticFriction, staticFriction);
}