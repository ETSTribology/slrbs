#pragma once

#include "friction/FrictionStrategy.h"
#include <cmath>

/**
 * Implementation of isotropic Coulomb friction model.
 */
class IsotropicCoulombFriction : public FrictionStrategy {
public:
    std::string getName() const override {
        return "IsotropicCoulomb";
    }
    
    void applyFrictionConstraints(
        Eigen::Vector3f& lambda,
        const Eigen::Matrix3f& A,
        const Eigen::Vector3f& b,
        const Contact* contact,
        float velThreshold = 0.1f) const override;
        
    void computeBounds(
        const Contact* contact,
        Eigen::VectorXf& lower,
        Eigen::VectorXf& upper,
        int index) const override;
        
    std::unique_ptr<FrictionStrategy> clone() const override {
        return std::make_unique<IsotropicCoulombFriction>(*this);
    }
};

/**
 * Implementation of smooth friction transition model.
 */
class SmoothFrictionTransition : public FrictionStrategy {
public:
    SmoothFrictionTransition(float staticToKineticRatio = 0.8f)
        : m_staticToKineticRatio(staticToKineticRatio) {}
        
    std::string getName() const override {
        return "SmoothTransition";
    }
    
    void applyFrictionConstraints(
        Eigen::Vector3f& lambda,
        const Eigen::Matrix3f& A,
        const Eigen::Vector3f& b,
        const Contact* contact,
        float velThreshold = 0.1f) const override;
        
    void computeBounds(
        const Contact* contact,
        Eigen::VectorXf& lower,
        Eigen::VectorXf& upper,
        int index) const override;
        
    std::unique_ptr<FrictionStrategy> clone() const override {
        return std::make_unique<SmoothFrictionTransition>(*this);
    }
    
    /**
     * Set the ratio of kinetic to static friction coefficient.
     * 
     * @param ratio Ratio (typically 0.7-0.9)
     */
    void setStaticToKineticRatio(float ratio) {
        m_staticToKineticRatio = ratio;
    }
    
    /**
     * Get the current static to kinetic friction ratio.
     * 
     * @return Current ratio
     */
    float getStaticToKineticRatio() const {
        return m_staticToKineticRatio;
    }
    
private:
    float m_staticToKineticRatio;
};

/**
 * Implementation of anisotropic friction model.
 */
class AnisotropicFriction : public FrictionStrategy {
public:
    AnisotropicFriction(float ratioT2ToT1 = 0.7f)
        : m_ratioT2ToT1(ratioT2ToT1) {}
        
    std::string getName() const override {
        return "Anisotropic";
    }
    
    void applyFrictionConstraints(
        Eigen::Vector3f& lambda,
        const Eigen::Matrix3f& A,
        const Eigen::Vector3f& b,
        const Contact* contact,
        float velThreshold = 0.1f) const override;
        
    void computeBounds(
        const Contact* contact,
        Eigen::VectorXf& lower,
        Eigen::VectorXf& upper,
        int index) const override;
        
    std::unique_ptr<FrictionStrategy> clone() const override {
        return std::make_unique<AnisotropicFriction>(*this);
    }
    
    /**
     * Set the ratio of friction coefficients in the T2 vs T1 directions.
     * 
     * @param ratio Ratio (typically less than 1.0)
     */
    void setDirectionalRatio(float ratio) {
        m_ratioT2ToT1 = ratio;
    }
    
    /**
     * Get the current directional friction ratio.
     * 
     * @return Current ratio
     */
    float getDirectionalRatio() const {
        return m_ratioT2ToT1;
    }
    
private:
    float m_ratioT2ToT1;
};

/**
 * Implementation of Stribeck friction model with velocity-dependent friction.
 */
class StribeckFriction : public FrictionStrategy {
public:
    StribeckFriction(float staticMu = 1.0f, float kineticMu = 0.8f, float stribeckVelocity = 0.1f, float stribeckExponent = 2.0f)
        : m_staticMu(staticMu), m_kineticMu(kineticMu), m_stribeckVelocity(stribeckVelocity), m_stribeckExponent(stribeckExponent) {}
        
    std::string getName() const override {
        return "Stribeck";
    }
    
    void applyFrictionConstraints(
        Eigen::Vector3f& lambda,
        const Eigen::Matrix3f& A,
        const Eigen::Vector3f& b,
        const Contact* contact,
        float velThreshold = 0.1f) const override;
        
    void computeBounds(
        const Contact* contact,
        Eigen::VectorXf& lower,
        Eigen::VectorXf& upper,
        int index) const override;
        
    std::unique_ptr<FrictionStrategy> clone() const override {
        return std::make_unique<StribeckFriction>(*this);
    }
    
    /**
     * Set the Stribeck model parameters.
     * 
     * @param staticMu Static friction coefficient
     * @param kineticMu Kinetic friction coefficient
     * @param stribeckVelocity Characteristic Stribeck velocity
     * @param stribeckExponent Stribeck curve exponent
     */
    void setParameters(float staticMu, float kineticMu, float stribeckVelocity, float stribeckExponent) {
        m_staticMu = staticMu;
        m_kineticMu = kineticMu;
        m_stribeckVelocity = stribeckVelocity;
        m_stribeckExponent = stribeckExponent;
    }
    
private:
    float m_staticMu;
    float m_kineticMu;
    float m_stribeckVelocity;
    float m_stribeckExponent;
    
    /**
     * Calculate the Stribeck friction coefficient for a given velocity.
     */
    float computeStribeckCoefficient(float velocity) const {
        const float absVel = std::abs(velocity);
        return m_kineticMu + (m_staticMu - m_kineticMu) * std::exp(-std::pow(absVel / m_stribeckVelocity, m_stribeckExponent));
    }
};