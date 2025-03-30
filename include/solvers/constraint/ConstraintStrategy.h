#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include "contact/Contact.h"
#include "joints/Joint.h"
#include "utils/Types.h"

/**
 * Base class for constraint handling strategies.
 * 
 * This class defines the interface for different approaches to constraint enforcement,
 * complementing the friction models to provide a complete constraint solution.
 */
class ConstraintStrategy {
public:
    virtual ~ConstraintStrategy() = default;
    
    /**
     * Get the name of this constraint strategy.
     * 
     * @return Strategy name
     */
    virtual std::string getName() const = 0;
    
    /**
     * Apply joint constraint stabilization.
     * 
     * @param joint The joint to stabilize
     * @param h Time step size
     * @param lambda Current constraint impulse vector
     * @param A Constraint matrix
     * @param b RHS vector
     */
    virtual void stabilizeJoint(
        Joint* joint,
        float h,
        Eigen::VectorXf& lambda,
        const Eigen::MatrixXf& A,
        const Eigen::VectorXf& b) const = 0;
    
    /**
     * Apply contact constraint stabilization.
     * 
     * @param contact The contact to stabilize
     * @param h Time step size
     * @param lambda Current constraint impulse vector
     * @param A Constraint matrix
     * @param b RHS vector
     */
    virtual void stabilizeContact(
        Contact* contact,
        float h,
        Eigen::Vector3f& lambda,
        const Eigen::Matrix3f& A,
        const Eigen::Vector3f& b) const = 0;
    
    /**
     * Create a clone of this strategy.
     * 
     * @return Unique pointer to a new instance of this strategy
     */
    virtual std::unique_ptr<ConstraintStrategy> clone() const = 0;
};

/**
 * Implementation of Baumgarte stabilization for constraints.
 */
class BaumgarteStabilization : public ConstraintStrategy {
public:
    BaumgarteStabilization(float alpha = 0.2f, float beta = 0.9f)
        : m_alpha(alpha), m_beta(beta) {}
    
    std::string getName() const override {
        return "Baumgarte";
    }
    
    void stabilizeJoint(
        Joint* joint,
        float h,
        Eigen::VectorXf& lambda,
        const Eigen::MatrixXf& A,
        const Eigen::VectorXf& b) const override;
    
    void stabilizeContact(
        Contact* contact,
        float h,
        Eigen::Vector3f& lambda,
        const Eigen::Matrix3f& A,
        const Eigen::Vector3f& b) const override;
    
    std::unique_ptr<ConstraintStrategy> clone() const override {
        return std::make_unique<BaumgarteStabilization>(*this);
    }
    
    /**
     * Set the stabilization parameters.
     * 
     * @param alpha Position correction factor
     * @param beta Velocity correction factor
     */
    void setParameters(float alpha, float beta) {
        m_alpha = alpha;
        m_beta = beta;
    }
    
    float getAlpha() const { return m_alpha; }
    float getBeta() const { return m_beta; }
    
private:
    float m_alpha; // Position correction factor
    float m_beta;  // Velocity correction factor
};

/**
 * Implementation of post-stabilization for constraints.
 */
class PostStabilization : public ConstraintStrategy {
public:
    PostStabilization(float correctionFactor = 0.8f, int maxIterations = 5)
        : m_correctionFactor(correctionFactor), m_maxIterations(maxIterations) {}
    
    std::string getName() const override {
        return "PostStabilization";
    }
    
    void stabilizeJoint(
        Joint* joint,
        float h,
        Eigen::VectorXf& lambda,
        const Eigen::MatrixXf& A,
        const Eigen::VectorXf& b) const override;
    
    void stabilizeContact(
        Contact* contact,
        float h,
        Eigen::Vector3f& lambda,
        const Eigen::Matrix3f& A,
        const Eigen::Vector3f& b) const override;
    
    std::unique_ptr<ConstraintStrategy> clone() const override {
        return std::make_unique<PostStabilization>(*this);
    }
    
    /**
     * Set the stabilization parameters.
     * 
     * @param correctionFactor Factor controlling position correction strength
     * @param maxIterations Maximum number of stabilization iterations
     */
    void setParameters(float correctionFactor, int maxIterations) {
        m_correctionFactor = correctionFactor;
        m_maxIterations = maxIterations;
    }
    
    float getCorrectionFactor() const { return m_correctionFactor; }
    int getMaxIterations() const { return m_maxIterations; }
    
private:
    float m_correctionFactor; // Position correction strength
    int m_maxIterations;      // Maximum stabilization iterations
};

/**
 * Implementation of position-based dynamics for constraints.
 */
class PositionBasedDynamics : public ConstraintStrategy {
public:
    PositionBasedDynamics(float alpha = 0.9f, float beta = 0.2f)
        : m_alpha(alpha), m_beta(beta) {}
    
    std::string getName() const override {
        return "PositionBasedDynamics";
    }
    
    void stabilizeJoint(
        Joint* joint,
        float h,
        Eigen::VectorXf& lambda,
        const Eigen::MatrixXf& A,
        const Eigen::VectorXf& b) const override;
    
    void stabilizeContact(
        Contact* contact,
        float h,
        Eigen::Vector3f& lambda,
        const Eigen::Matrix3f& A,
        const Eigen::Vector3f& b) const override;
    
    std::unique_ptr<ConstraintStrategy> clone() const override {
        return std::make_unique<PositionBasedDynamics>(*this);
    }
    
    /**
     * Set the stabilization parameters.
     * 
     * @param alpha Stiffness parameter
     * @param beta Damping parameter
     */
    void setParameters(float alpha, float beta) {
        m_alpha = alpha;
        m_beta = beta;
    }
    
    float getAlpha() const { return m_alpha; }
    float getBeta() const { return m_beta; }
    
private:
    float m_alpha; // Stiffness parameter
    float m_beta;  // Damping parameter
};

/**
 * Registry for constraint stabilization strategies.
 */
class ConstraintStrategyRegistry {
public:
    using StrategyCreator = std::function<std::unique_ptr<ConstraintStrategy>()>;
    
    /**
     * Get the singleton instance.
     */
    static ConstraintStrategyRegistry& getInstance() {
        static ConstraintStrategyRegistry instance;
        return instance;
    }
    
    /**
     * Initialize the registry with standard stabilization strategies.
     */
    void initializeStandardStrategies();
    
    /**
     * Register a new stabilization strategy.
     */
    bool registerStrategy(const std::string& name, StrategyCreator creator);
    
    /**
     * Check if a strategy is registered.
     */
    bool isStrategyRegistered(const std::string& name) const;
    
    /**
     * Create a new instance of a registered strategy.
     */
    std::unique_ptr<ConstraintStrategy> createStrategy(const std::string& name) const;
    
    /**
     * Get all registered strategy names.
     */
    std::vector<std::string> getRegisteredStrategyNames() const;
    
private:
    // Private constructor for singleton
    ConstraintStrategyRegistry() = default;
    
    // Map of strategy names to creator functions
    std::unordered_map<std::string, StrategyCreator> m_strategyFactories;
};