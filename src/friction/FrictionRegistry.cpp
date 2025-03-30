#pragma once

#include "solvers/Solver.h"
#include "solvers/SolverFactory.h"
#include "friction/FrictionStrategy.h"
#include "friction/FrictionRegistry.h"
#include <memory>
#include <string>
#include <optional>
#include <functional>

/**
 * Builder for configuring solvers in a fluent manner.
 * 
 * This class implements the Builder pattern to allow for fluent configuration
 * of solver instances with various parameters.
 */
class SolverBuilder {
public:
    /**
     * Create a new builder for a specific solver type.
     * 
     * @param solverType Type of solver to create
     * @param rigidBodySystem The rigid body system to solve for
     */
    SolverBuilder(const std::string& solverType, RigidBodySystem* rigidBodySystem)
        : m_solverType(solverType), m_rigidBodySystem(rigidBodySystem) {}
    
    /**
     * Set the maximum number of iterations.
     * 
     * @param maxIter Maximum number of iterations
     * @return Reference to this builder for method chaining
     */
    SolverBuilder& withMaxIterations(int maxIter) {
        m_maxIter = maxIter;
        return *this;
    }
    
    /**
     * Set the number of threads for parallel computation.
     * 
     * @param numThreads Number of threads to use
     * @return Reference to this builder for method chaining
     */
    SolverBuilder& withThreads(int numThreads) {
        m_numThreads = numThreads;
        return *this;
    }
    
    /**
     * Enable or disable parallel computation.
     * 
     * @param enable Whether to enable parallel computation
     * @return Reference to this builder for method chaining
     */
    SolverBuilder& withParallelComputation(bool enable) {
        m_parallelEnabled = enable;
        return *this;
    }
    
    /**
     * Enable or disable performance measurement.
     * 
     * @param enable Whether to enable performance measurement
     * @return Reference to this builder for method chaining
     */
    SolverBuilder& withPerformanceMeasurement(bool enable) {
        m_measurePerformance = enable;
        return *this;
    }
    
    /**
     * Set the friction strategy to use.
     * 
     * @param strategyName Name of the friction strategy
     * @return Reference to this builder for method chaining
     */
    SolverBuilder& withFrictionStrategy(const std::string& strategyName) {
        auto& registry = FrictionRegistry::getInstance();
        if (registry.isStrategyRegistered(strategyName)) {
            m_frictionStrategy = strategyName;
        }
        return *this;
    }
    
    /**
     * Set a custom friction strategy to use.
     * 
     * @param strategy Unique pointer to a custom friction strategy
     * @return Reference to this builder for method chaining
     */
    SolverBuilder& withCustomFrictionStrategy(std::unique_ptr<FrictionStrategy> strategy) {
        if (strategy) {
            m_customFrictionStrategy = std::move(strategy);
        }
        return *this;
    }
    
    /**
     * Set the friction velocity threshold.
     * 
     * @param threshold Velocity threshold for friction transition
     * @return Reference to this builder for method chaining
     */
    SolverBuilder& withFrictionVelocityThreshold(float threshold) {
        m_frictionVelocityThreshold = threshold;
        return *this;
    }
    
    /**
     * Enable or disable smooth friction transition.
     * 
     * @param enable Whether to enable smooth friction transition
     * @return Reference to this builder for method chaining
     */
    SolverBuilder& withSmoothFriction(bool enable) {
        m_useSmoothFriction = enable;
        if (enable) {
            m_frictionStrategy = "SmoothTransition";
        }
        return *this;
    }
    
    /**
     * Set the ratio of static to kinetic friction for smooth friction models.
     * 
     * @param ratio Ratio of static to kinetic friction (typically 0.7-0.9)
     * @return Reference to this builder for method chaining
     */
    SolverBuilder& withStaticToKineticRatio(float ratio) {
        m_staticToKineticRatio = ratio;
        return *this;
    }
    
    /**
     * Set the solver error tolerance.
     * 
     * @param tolerance Error tolerance for convergence checks
     * @return Reference to this builder for method chaining
     */
    SolverBuilder& withTolerance(float tolerance) {
        m_tolerance = tolerance;
        return *this;
    }
    
    /**
     * Set the warm starting factor.
     * 
     * @param factor Warm starting factor (0.0-1.0, 0.0 disables warm starting)
     * @return Reference to this builder for method chaining
     */
    SolverBuilder& withWarmStartingFactor(float factor) {
        m_warmStartingFactor = factor;
        return *this;
    }
    
    /**
     * Enable or disable sequential impulses ordering.
     * 
     * @param enable Whether to enable sequential impulses ordering
     * @return Reference to this builder for method chaining
     */
    SolverBuilder& withSequentialImpulses(bool enable) {
        m_useSequentialImpulses = enable;
        return *this;
    }
    
    /**
     * Configure the solver to use a specific constraint relaxation approach.
     * 
     * @param approach Name of the relaxation approach (e.g., "SOR", "Jacobi")
     * @param omega Relaxation parameter (0.0-2.0, typically around 1.3)
     * @return Reference to this builder for method chaining
     */
    SolverBuilder& withRelaxation(const std::string& approach, float omega = 1.3f) {
        m_relaxationApproach = approach;
        m_relaxationOmega = omega;
        return *this;
    }
    
    /**
     * Enable or disable data export.
     * 
     * @param enable Whether to enable data export
     * @param path Base path for export files
     * @return Reference to this builder for method chaining
     */
    SolverBuilder& withDataExport(bool enable, const std::string& path = "") {
        m_exportEnabled = enable;
        if (!path.empty()) {
            m_exportPath = path;
        }
        return *this;
    }
    
    /**
     * Set a custom callback function for post-solve analysis.
     * 
     * @param callback Function to call after each solve operation
     * @return Reference to this builder for method chaining
     */
    SolverBuilder& withPostSolveCallback(std::function<void(Solver*, float)> callback) {
        if (callback) {
            m_postSolveCallback = callback;
        }
        return *this;
    }
    
    /**
     * Build and configure the solver.
     * 
     * @return Unique pointer to the configured solver
     */
    std::unique_ptr<Solver> build() {
        // Create the solver
        auto solver = SolverFactory::createSolver(m_solverType, m_rigidBodySystem);
        if (!solver) {
            return nullptr;
        }
        
        // Configure common parameters
        if (m_maxIter > 0) {
            solver->setMaxIter(m_maxIter);
        }
        
        if (m_numThreads > 0) {
            solver->setNumThreads(m_numThreads);
        }
        
        solver->enableParallelComputation(m_parallelEnabled);
        solver->enablePerformanceMeasurement(m_measurePerformance);
        
        // Configure solver-specific parameters
        configureSolverSpecific(solver.get());
        
        return solver;
    }
    
private:
    /**
     * Configure solver-specific parameters based on the solver type.
     * 
     * @param solver Pointer to the solver to configure
     */
    void configureSolverSpecific(Solver* solver);
    
    std::string m_solverType;
    RigidBodySystem* m_rigidBodySystem;
    
    // Common parameters
    int m_maxIter = -1;
    int m_numThreads = -1;
    bool m_parallelEnabled = true;
    bool m_measurePerformance = false;
    std::optional<float> m_tolerance;
    std::optional<float> m_warmStartingFactor;
    std::optional<bool> m_useSequentialImpulses;
    std::optional<std::string> m_relaxationApproach;
    std::optional<float> m_relaxationOmega;
    
    // Friction parameters
    std::optional<std::string> m_frictionStrategy;
    std::unique_ptr<FrictionStrategy> m_customFrictionStrategy;
    std::optional<float> m_frictionVelocityThreshold;
    std::optional<bool> m_useSmoothFriction;
    std::optional<float> m_staticToKineticRatio;
    
    // Export parameters
    std::optional<bool> m_exportEnabled;
    std::optional<std::string> m_exportPath;
    
    // Custom callbacks
    std::optional<std::function<void(Solver*, float)>> m_postSolveCallback;
};