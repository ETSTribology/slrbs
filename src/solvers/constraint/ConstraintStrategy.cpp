#include "solvers/ConstraintStrategy.h"
#include "contact/Contact.h"
#include "joints/Joint.h"
#include <cmath>

// BaumgarteStabilization implementation
void BaumgarteStabilization::stabilizeJoint(
    Joint* joint,
    float h,
    Eigen::VectorXf& lambda,
    const Eigen::MatrixXf& A,
    const Eigen::VectorXf& b) const 
{
    // Skip if constraint is already satisfied
    if (joint->phi.norm() < 1e-6f) {
        return;
    }

    // Compute stabilization correction
    Eigen::VectorXf correctionImpulse = -(m_alpha / h) * joint->phi - m_beta * joint->phiDot;
    
    // Apply correction impulse
    lambda += A.ldlt().solve(correctionImpulse);
    
    // Update joint velocities and positions
    joint->applyImpulse(lambda);
}

void BaumgarteStabilization::stabilizeContact(
    Contact* contact,
    float h,
    Eigen::Vector3f& lambda,
    const Eigen::Matrix3f& A,
    const Eigen::Vector3f& b) const 
{
    // Only stabilize penetrating contacts
    if (contact->phi >= 0.0f) {
        return;
    }

    // Compute normal correction impulse (only in normal direction)
    float correctionImpulse = -(m_alpha / h) * contact->phi - m_beta * contact->relativeVelocity.dot(contact->normal);
    
    // Apply correction - only to normal component (index 0)
    Eigen::Vector3f deltaLambda = Eigen::Vector3f::Zero();
    deltaLambda(0) = correctionImpulse;
    
    // Solve for the actual correction considering coupling
    Eigen::Vector3f actualDelta = A.ldlt().solve(deltaLambda);
    
    // Add the correction impulse
    lambda += actualDelta;
    
    // Ensure non-negativity of normal impulse
    lambda(0) = std::max(0.0f, lambda(0));
}

// PostStabilization implementation
void PostStabilization::stabilizeJoint(
    Joint* joint,
    float h,
    Eigen::VectorXf& lambda,
    const Eigen::MatrixXf& A,
    const Eigen::VectorXf& b) const 
{
    // Original lambda for later comparison
    Eigen::VectorXf originalLambda = lambda;
    
    for (int iter = 0; iter < m_maxIterations; ++iter) {
        // Check if constraint is already satisfied
        if (joint->phi.norm() < 1e-6f) {
            break;
        }
        
        // Compute position-level correction
        Eigen::VectorXf deltaX = -m_correctionFactor * joint->phi;
        
        // Convert to impulse correction using effective mass
        Eigen::VectorXf correctionImpulse = (1.0f / h) * A.ldlt().solve(deltaX);
        
        // Apply correction impulse
        joint->applyPositionCorrection(deltaX);
        
        // Update lambda to account for the applied correction
        lambda += correctionImpulse;
    }
    
    // Compute how much work the stabilization did
    float stabilizationWork = (lambda - originalLambda).norm();
    
    // For debugging or analysis purposes
    // std::cout << "Joint stabilization work: " << stabilizationWork << std::endl;
}

void PostStabilization::stabilizeContact(
    Contact* contact,
    float h,
    Eigen::Vector3f& lambda,
    const Eigen::Matrix3f& A,
    const Eigen::Vector3f& b) const 
{
    // Only stabilize penetrating contacts
    if (contact->phi >= 0.0f) {
        return;
    }
    
    // Original lambda for later comparison
    Eigen::Vector3f originalLambda = lambda;
    
    for (int iter = 0; iter < m_maxIterations; ++iter) {
        // Check if penetration is resolved
        if (contact->phi >= -1e-6f) {
            break;
        }
        
        // Compute position-level correction (only in normal direction)
        float deltaPhi = -m_correctionFactor * contact->phi;
        
        // Convert to impulse correction using effective mass
        Eigen::Vector3f deltaLambda = Eigen::Vector3f::Zero();
        deltaLambda(0) = (1.0f / h) * deltaPhi / A(0, 0); // Simplified for efficiency
        
        // Apply correction
        contact->applyPositionCorrection(deltaPhi * contact->normal);
        
        // Update lambda
        lambda += deltaLambda;
        
        // Ensure non-negativity of normal impulse
        lambda(0) = std::max(0.0f, lambda(0));
    }
    
    // Compute how much work the stabilization did
    float stabilizationWork = (lambda - originalLambda).norm();
    
    // For debugging or analysis purposes
    // std::cout << "Contact stabilization work: " << stabilizationWork << std::endl;
}

// PositionBasedDynamics implementation
void PositionBasedDynamics::stabilizeJoint(
    Joint* joint,
    float h,
    Eigen::VectorXf& lambda,
    const Eigen::MatrixXf& A,
    const Eigen::VectorXf& b) const 
{
    // Compute constraint violation
    float violation = joint->phi.norm();
    
    // Skip if constraint is already satisfied
    if (violation < 1e-6f) {
        return;
    }
    
    // Compute position correction using PBD
    Eigen::VectorXf deltaPos = -joint->phi;
    
    // Compute stiffness based on time step
    float k = 1.0f - std::pow(1.0f - m_alpha, h);
    
    // Scale correction by stiffness
    deltaPos *= k;
    
    // Apply position correction
    joint->applyPositionBasedCorrection(deltaPos, m_beta);
    
    // Update velocities from positions if needed
    joint->updateVelocitiesFromPositions(h);
}

void PositionBasedDynamics::stabilizeContact(
    Contact* contact,
    float h,
    Eigen::Vector3f& lambda,
    const Eigen::Matrix3f& A,
    const Eigen::Vector3f& b) const 
{
    // Only stabilize penetrating contacts
    if (contact->phi >= 0.0f) {
        return;
    }
    
    // Compute stiffness based on time step
    float k = 1.0f - std::pow(1.0f - m_alpha, h);
    
    // Compute position correction
    float deltaPos = -k * contact->phi;
    
    // Apply position correction
    contact->applyPositionBasedCorrection(deltaPos * contact->normal, m_beta);
    
    // Update velocities from positions if needed
    contact->updateVelocitiesFromPositions(h);
}

// ConstraintStrategyRegistry implementation
void ConstraintStrategyRegistry::initializeStandardStrategies() {
    // Register standard strategies
    registerStrategy("Baumgarte", []() {
        return std::make_unique<BaumgarteStabilization>();
    });
    
    registerStrategy("PostStabilization", []() {
        return std::make_unique<PostStabilization>();
    });
    
    registerStrategy("PBD", []() {
        return std::make_unique<PositionBasedDynamics>();
    });
}

bool ConstraintStrategyRegistry::registerStrategy(const std::string& name, StrategyCreator creator) {
    if (m_strategyFactories.find(name) != m_strategyFactories.end()) {
        return false; // Already registered
    }
    
    m_strategyFactories[name] = creator;
    return true;
}

bool ConstraintStrategyRegistry::isStrategyRegistered(const std::string& name) const {
    return m_strategyFactories.find(name) != m_strategyFactories.end();
}

std::unique_ptr<ConstraintStrategy> ConstraintStrategyRegistry::createStrategy(const std::string& name) const {
    auto it = m_strategyFactories.find(name);
    if (it == m_strategyFactories.end()) {
        return nullptr; // Not registered
    }
    
    return it->second();
}

std::vector<std::string> ConstraintStrategyRegistry::getRegisteredStrategyNames() const {
    std::vector<std::string> names;
    names.reserve(m_strategyFactories.size());
    
    for (const auto& pair : m_strategyFactories) {
        names.push_back(pair.first);
    }
    
    return names;
}