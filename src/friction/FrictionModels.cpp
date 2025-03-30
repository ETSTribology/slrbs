#include "friction/FrictionModels.h"
#include "contact/Contact.h"
#include <algorithm>
#include <cmath>

// Helper function to compute smooth friction coefficient
static float computeSmoothFrictionCoefficient(
    float relVel,
    float staticMu,
    float kineticMu,
    float velThreshold) 
{
    if (std::abs(relVel) < 1e-6f) {
        return staticMu;
    }
    
    const float absVel = std::abs(relVel);
    if (absVel >= velThreshold) {
        return kineticMu;
    }
    
    // Smooth transition using cubic Hermite spline
    const float t = absVel / velThreshold;
    const float t2 = t * t;
    const float t3 = t2 * t;
    
    // Hermite basis functions
    const float h1 = 2.0f * t3 - 3.0f * t2 + 1.0f;
    const float h2 = -2.0f * t3 + 3.0f * t2;
    
    // Interpolate between static and kinetic friction
    return staticMu * h1 + kineticMu * h2;
}

// IsotropicCoulombFriction implementation
void IsotropicCoulombFriction::applyFrictionConstraints(
    Eigen::Vector3f& lambda,
    const Eigen::Matrix3f& A,
    const Eigen::Vector3f& b,
    const Contact* contact,
    float velThreshold) const 
{
    // Normal impulse is projected to [0, inf]
    lambda(0) = std::max(0.0f, (b(0) - A(0, 1) * lambda(1) - A(0, 2) * lambda(2)) / (A(0, 0) + 1e-6f));
    
    // Friction impulse bounds
    const float mu = contact->mu;
    const float lowerBound = -mu * lambda(0);
    const float upperBound = mu * lambda(0);
    
    // Tangential impulses are projected to [-mu * lambda_n, mu * lambda_n]
    lambda(1) = std::max(lowerBound, std::min(upperBound, 
        (b(1) - A(1, 0) * lambda(0) - A(1, 2) * lambda(2)) / (A(1, 1) + 1e-6f)));
        
    lambda(2) = std::max(lowerBound, std::min(upperBound, 
        (b(2) - A(2, 0) * lambda(0) - A(2, 1) * lambda(1)) / (A(2, 2) + 1e-6f)));
}

void IsotropicCoulombFriction::computeBounds(
    const Contact* contact,
    Eigen::VectorXf& lower,
    Eigen::VectorXf& upper,
    int index) const
{
    // Normal component has [0, inf] bounds
    lower(index) = 0.0f;
    upper(index) = std::numeric_limits<float>::max();
    
    // Initially set friction bounds to 0, will be updated during solution
    // based on the normal force
    lower(index + 1) = -contact->mu;  // Will be multiplied by normal force
    lower(index + 2) = -contact->mu;  // Will be multiplied by normal force
    upper(index + 1) = contact->mu;   // Will be multiplied by normal force
    upper(index + 2) = contact->mu;   // Will be multiplied by normal force
}

// SmoothFrictionTransition implementation
void SmoothFrictionTransition::applyFrictionConstraints(
    Eigen::Vector3f& lambda,
    const Eigen::Matrix3f& A,
    const Eigen::Vector3f& b,
    const Contact* contact,
    float velThreshold) const 
{
    // Normal impulse is projected to [0, inf]
    lambda(0) = std::max(0.0f, (b(0) - A(0, 1) * lambda(1) - A(0, 2) * lambda(2)) / (A(0, 0) + 1e-6f));
    
    // Compute relative tangential velocity
    const float relVelT1 = contact->relativeVelocity.dot(contact->t1);
    const float relVelT2 = contact->relativeVelocity.dot(contact->t2);
    const float relTangVel = std::sqrt(relVelT1 * relVelT1 + relVelT2 * relVelT2);
    
    // Use smooth friction model with different static and kinetic coefficients
    const float staticMu = contact->mu;
    const float kineticMu = staticMu * m_staticToKineticRatio;
    const float effectiveMu = computeSmoothFrictionCoefficient(
        relTangVel, staticMu, kineticMu, velThreshold);
    
    // Friction impulse bounds
    const float lowerBound = -effectiveMu * lambda(0);
    const float upperBound = effectiveMu * lambda(0);
    
    // Tangential impulses are projected to bounds
    lambda(1) = std::max(lowerBound, std::min(upperBound, 
        (b(1) - A(1, 0) * lambda(0) - A(1, 2) * lambda(2)) / (A(1, 1) + 1e-6f)));
        
    lambda(2) = std::max(lowerBound, std::min(upperBound, 
        (b(2) - A(2, 0) * lambda(0) - A(2, 1) * lambda(1)) / (A(2, 2) + 1e-6f)));
}

void SmoothFrictionTransition::computeBounds(
    const Contact* contact,
    Eigen::VectorXf& lower,
    Eigen::VectorXf& upper,
    int index) const
{
    // Use the maximum (static) friction for initial bounds estimation
    // The actual friction will be velocity-dependent during solution
    lower(index) = 0.0f;
    upper(index) = std::numeric_limits<float>::max();
    
    lower(index + 1) = -contact->mu;  // Will be adjusted during solution
    lower(index + 2) = -contact->mu;  // Will be adjusted during solution
    upper(index + 1) = contact->mu;   // Will be adjusted during solution
    upper(index + 2) = contact->mu;   // Will be adjusted during solution
}

// AnisotropicFriction implementation
void AnisotropicFriction::applyFrictionConstraints(
    Eigen::Vector3f& lambda,
    const Eigen::Matrix3f& A,
    const Eigen::Vector3f& b,
    const Contact* contact,
    float velThreshold) const 
{
    // Normal impulse is projected to [0, inf]
    lambda(0) = std::max(0.0f, (b(0) - A(0, 1) * lambda(1) - A(0, 2) * lambda(2)) / (A(0, 0) + 1e-6f));
    
    // Different friction coefficients for different directions
    const float mu1 = contact->mu;
    const float mu2 = contact->mu * m_ratioT2ToT1;
    
    // Friction impulse bounds for each direction
    const float lowerBound1 = -mu1 * lambda(0);
    const float upperBound1 = mu1 * lambda(0);
    const float lowerBound2 = -mu2 * lambda(0);
    const float upperBound2 = mu2 * lambda(0);
    
    // Tangential impulses are projected to direction-specific bounds
    lambda(1) = std::max(lowerBound1, std::min(upperBound1, 
        (b(1) - A(1, 0) * lambda(0) - A(1, 2) * lambda(2)) / (A(1, 1) + 1e-6f)));
        
    lambda(2) = std::max(lowerBound2, std::min(upperBound2, 
        (b(2) - A(2, 0) * lambda(0) - A(2, 1) * lambda(1)) / (A(2, 2) + 1e-6f)));
}

void AnisotropicFriction::computeBounds(
    const Contact* contact,
    Eigen::VectorXf& lower,
    Eigen::VectorXf& upper,
    int index) const
{
    lower(index) = 0.0f;
    upper(index) = std::numeric_limits<float>::max();
    
    const float mu1 = contact->mu;
    const float mu2 = contact->mu * m_ratioT2ToT1;
    
    lower(index + 1) = -mu1;  // T1 direction
    lower(index + 2) = -mu2;  // T2 direction
    upper(index + 1) = mu1;   // T1 direction
    upper(index + 2) = mu2;   // T2 direction
}

// StribeckFriction implementation
void StribeckFriction::applyFrictionConstraints(
    Eigen::Vector3f& lambda,
    const Eigen::Matrix3f& A,
    const Eigen::Vector3f& b,
    const Contact* contact,
    float velThreshold) const 
{
    // Normal impulse is projected to [0, inf]
    lambda(0) = std::max(0.0f, (b(0) - A(0, 1) * lambda(1) - A(0, 2) * lambda(2)) / (A(0, 0) + 1e-6f));
    
    // Compute relative tangential velocity
    const float relVelT1 = contact->relativeVelocity.dot(contact->t1);
    const float relVelT2 = contact->relativeVelocity.dot(contact->t2);
    
    // Compute Stribeck friction coefficients for each direction
    const float mu1 = computeStribeckCoefficient(relVelT1);
    const float mu2 = computeStribeckCoefficient(relVelT2);
    
    // Friction impulse bounds for each direction
    const float lowerBound1 = -mu1 * lambda(0);
    const float upperBound1 = mu1 * lambda(0);
    const float lowerBound2 = -mu2 * lambda(0);
    const float upperBound2 = mu2 * lambda(0);
    
    // Tangential impulses are projected to direction-specific bounds
    lambda(1) = std::max(lowerBound1, std::min(upperBound1, 
        (b(1) - A(1, 0) * lambda(0) - A(1, 2) * lambda(2)) / (A(1, 1) + 1e-6f)));
        
    lambda(2) = std::max(lowerBound2, std::min(upperBound2, 
        (b(2) - A(2, 0) * lambda(0) - A(2, 1) * lambda(1)) / (A(2, 2) + 1e-6f)));
}

void StribeckFriction::computeBounds(
    const Contact* contact,
    Eigen::VectorXf& lower,
    Eigen::VectorXf& upper,
    int index) const
{
    // Use the maximum (static) friction for initial bounds estimation
    // The actual friction will be velocity-dependent during solution
    lower(index) = 0.0f;
    upper(index) = std::numeric_limits<float>::max();
    
    lower(index + 1) = -m_staticMu;  // Will be adjusted during solution
    lower(index + 2) = -m_staticMu;  // Will be adjusted during solution
    upper(index + 1) = m_staticMu;   // Will be adjusted during solution
    upper(index + 2) = m_staticMu;   // Will be adjusted during solution
}